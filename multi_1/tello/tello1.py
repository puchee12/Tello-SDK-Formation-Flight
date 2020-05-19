import socket
import threading
import time
from errno import ENETUNREACH

import cv2 as cv
import numpy as np

class Tello1:
	'''
	The Tello class interfaces with the Tello drone based on the Tello SDK v1.3:
	https://dl-cdn.ryzerobotics.com/downloads/tello/20180910/Tello%20SDK%20Documentation%20EN_1.3.pdf
	'''

	# Tello address based on IP and port
	TELLO_IP = '192.168.10.1'
	TELLO_PORT = 8889
	TELLO_ADDRESS = (TELLO_IP, TELLO_PORT)

	# Local address
	LOCAL_IP = '0.0.0.0'
	LOCAL_PORT = 8889
	LOCAL_ADDRESS = (LOCAL_IP, LOCAL_PORT)

	# Port for receiving video
	VIDEO_PORT = 11111
	VIDEO_ADDRESS = (LOCAL_IP, VIDEO_PORT)

	# State address
	STATE_PORT = 8890
	STATE_ADDRESS = (LOCAL_IP, STATE_PORT)

	# Wait time between commands (s)
	command_timeout = 0.3

	# Address used by openCV function 'VideoCapture()' (pointer to camera)
	# Found at: https://github.com/damiafuentes/DJITelloPy/blob/master/djitellopy/tello.py
	camera_address = ('udp://@' + LOCAL_IP + ':' + str(VIDEO_PORT)) #'?overrun_nonfatal=1&fifo_size=5000'

	def __init__(self):
		'''Initialises the object, sockets and a response receiving thread.'''
		# Initialise variables
		self.response = None
		self.cap = None		# Video object
		self.frame = None   # Video frame
		self.streamon = False
		# Socket for commands, video and state. IPv4, with UDP
		# self.socket_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		# self.socket_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		# self.socket_state = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

		# Socket for commands, video and state. IPv4, with UDP
		self.socket_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.socket_cmd.setsockopt(socket.SOL_SOCKET, 25, 'wlx503eaa2e6985'.encode())
		self.socket_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.socket_video.setsockopt(socket.SOL_SOCKET, 25, 'wlx503eaa2e6985'.encode())
		self.socket_state = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.socket_state.setsockopt(socket.SOL_SOCKET, 25, 'wlx503eaa2e6985'.encode())

		# Bind to enable communication and listening
		# (Video bind only required for H264Decoder in startVideoCapture method)
		self.socket_cmd.bind(Tello1.LOCAL_ADDRESS)	# comms
		self.socket_state.bind(Tello1.STATE_ADDRESS)	# listening for state

		# Receiver thread - Daemon True so it closes when main thread ends
		self.receive_thread = threading.Thread(target=self._udpReceive, 
			daemon=True)
		self.receive_thread.start()


		# Send a byte string "command" to initiate Tello's SDK mode
		try:
			self.sendCommand('command')
		except OSError as exc:
			# Prints a message and stops the program if no connection exists.		
			if exc.errno == ENETUNREACH:
				print("Network connection is not established. Wait for Wi-Fi" +
					" to finish connecting to Tello and retry.")
				print(exc)
				self.shutdown()
				raise SystemExit(0)

	def _udpReceive(self):
		'''Method runs as a thread to constantly receive responses.'''
		while True:
			try:
				self.response, _ = self.socket_cmd.recvfrom(1024)
				print('Response: ' + self.response.decode('utf-8'))
			except Exception as exc:
				print('Exception in _udpReceive:', exc)

	def startStateCapture(self):
		'''Creates and starts a thread for listening to incoming state.'''
		self.state_thread = threading.Thread(target=self._stateReceive,
			daemon=True)
		self.state_thread.start()
		time.sleep(0.01)	# Time to retrieve first state

	def _stateReceive(self):
		'''Method runs as a thread to constantly receive updated state.'''
		while True:
			try:
				self.raw_state, _ = self.socket_state.recvfrom(1024)
				self.raw_state = self.raw_state.decode('utf-8')
			except Exception as exc:
				print('Exception in _stateReceive:', exc)

	def readState(self):
		'''Parses the state string and returns it as a dictionary type

		The raw state has following form:
		"pitch:%d;roll:%d;yaw:%d;vgx:%d;vgy%d;vgz:%d;templ:%d;temph:%d;tof:%d;
		h:%d;bat:%d;baro:%.2f;time:%d;agx:%.2f;agy:%.2f;agz:%.2f;\r\n”

		The returned dictionary contains the following:
		{'pitch': %f,
		'roll': %f,
		'yaw': %f,
		'vx': %f,
		'vy': %f,
		'vz': %f,
		'templ': %f,
		'temph': %f,
		'tof': %f,
		'height': %f,
		'battery': %f,
		'baro': %f,
		'time': %f,
		'ax': %f,
		'ay': %f,
		'az': %f}
		'''
		try:
			# Parses the raw string by splitting at the semicolons, deleting
			# the last unnecessary element and creating a list of floats of the
			# numbers after the colon.
			self.state = self.raw_state.split(';')
			del self.state[-1]		# Delete last element: [....,'\r\n']
			self.state = [float(i.split(':')[1]) for i in self.state]

			# Create keys and zip with the floats to create a dictionary
			self.state_dict_keys = ('pitch','roll','yaw','vx','vy','vz','templ',
				'temph','tof','height','battery','baro','time','ax','ay','az')
			self.state_dict = dict(zip(self.state_dict_keys, self.state))

			return self.state_dict

		# If the stateReceive thread was just started then raw_state may not
		# exist yet
		except AttributeError as atr_error:
			print('Exception in readState:', atr_error)

	def startVideoCapture(self, method="H264Decoder"):
		'''
		Initiates video capture by starting the Tello camera, finding the 
		pointer to the video and starting the thread which updates each frame.
		The 'method' argument can either be 'H264Decoder' or 'OpenCV' which
		selects between 2 difference ways to retrieve and decode frames. The 
		'H264Decoder' is significantly faster hence it is the default.
		'''
		# Send "streamon" command to start streaming video
		self.sendCommandNoWait('streamon')
		self.streamon = True
		self.new_frame = False	# used to track when a new frame has arrived

		if method == "H264Decoder":
			# Bind video receiving socket
			self.socket_video.bind(Tello1.VIDEO_ADDRESS)

			# Start thread using H264Decoder method
			self.video_thread = threading.Thread(target=self._updateFrameH264, 
							daemon=True)
			self.video_thread.start()

			# Wait until first frame arrives
			while self.readFrame() is None:
				time.sleep(0.1)
		elif method == "OpenCV":
			# VideoCapture (OpenCV function) will bind the camera_address
			self.cap = cv.VideoCapture(Tello1.camera_address)
			self.ret, self.frame = self.cap.read()	# Manually read first frame

			# Start thread using OpenCV method
			self.video_thread = threading.Thread(target=self._updateFrameOpenCV, 
							daemon=True)
			self.video_thread.start()
		else:
			print('Invalid method, choose a correct method: "H264Decoder" or "OpenCV".')
			self.shutdown()
			raise SystemExit(0)

	def _updateFrameH264(self):
		'''Updates frames through a thread using the H264Decoder.'''
		# Initialise empty byte array to hold the frames
		packet_data = b""
		while self.streamon:
			try:
				# Receive frames through the video socket and add to packet_data
				res_string, _ = self.socket_video.recvfrom(2048)
				packet_data += res_string

				# res_string has a length of 1460 in each iteration except at 
				# end of frame where it is smaller.
				if len(res_string) != 1460:
					for frame in self._h264_decode(packet_data):
						# Convert the returned frame from BGR to RGB
						self.frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

					# Reset packet_data to empty for next frame
					packet_data = b""

			except socket.error as exc:
				print('Exception in _updateFrameH264:', exc)

	def _h264_decode(self, packet_data):
		'''Decodes raw H264 frame data to a decoded frame.'''
		res_frame_list = []
		frames = self.decoder.decode(packet_data)
		for framedata in frames:
			(frame, w, h, ls) = framedata
			if frame is not None:
				# print('frame size %i bytes, w %i, h %i, linesize %i' % (len(frame), w, h, ls))
				frame = np.fromstring(frame, dtype=np.ubyte, count=len(frame), sep='')
				frame = (frame.reshape((h, int(ls / 3), 3)))
				frame = frame[:, :w, :]
				res_frame_list.append(frame)

		# New frame fully decoded and ready therefore set flag to True
		self.new_frame = True
		return res_frame_list

	def _updateFrameOpenCV(self):
		'''Updates frame through a thread using the VideoCapture pointer.'''
		while self.streamon:
			try:
				self.ret, self.frame = self.cap.read()
				self.new_frame = True
			except Exception as exc:
				print('Exception in _updateFrame:', exc)

	def readFrame(self):
		'''Returns latest frame to the calling object.'''
		return self.frame

	def shutdown(self):
		'''Shutdown procedure, stop video capture and close sockets.'''
		if self.streamon:
			self.stopVideoCapture()
		self.closeSockets()

	def closeSockets(self):
		'''Closes all sockets.'''
		self.socket_cmd.close()
		self.socket_video.close()
		self.socket_state.close()

	def stopVideoCapture(self):
		'''Stops video capture by sending "streamoff" command.'''
		# if self.cap is not None:
		# 	print('Releasing stream')
		# 	self.cap.release()
		self.streamon = False
		self.sendCommandNoWait('streamoff')

	def sendCommand(self, command):
		'''
		Sends utf8 encoded command to the Tello and sleeps before sending
		next command to allow time for a response.
		'''
		print('--> Command sent:', command)
		self.socket_cmd.sendto(command.encode('utf8'), Tello1.TELLO_ADDRESS)
		time.sleep(Tello1.command_timeout)

	def sendCommandNoWait(self, command):
		'''Sends utf8 encoded command to the Tello with no waiting after.'''
		print('--> Command sent:', command)
		self.socket_cmd.sendto(command.encode('utf8'), Tello1.TELLO_ADDRESS)

	# Control Commands
	def takeoff(self):
		'''Sends takeoff command.'''
		return self.sendCommand('takeoff')

	def land(self):
		'''Sends land command.'''
		return self.sendCommand('land')

	def emergency(self):
		'''Stops all motors immediately.'''
		return self.sendCommand('emergency')

	def move(self, direction, distance):
		'''
		Move directions include up, down, left, right, forward, back 
		and distance ranges from 20 - 500 cm
		'''	
		return self.sendCommand(direction + ' ' + str(distance))

	def rotate(self, direction, degrees):
		'''Rotates cw or ccw for a maximum of 3600 degrees.'''
		return self.sendCommand(direction + ' ' + str(degrees))

	def curve(self, point1, point2, speed):
		point1 = str(point1)[1:-1]
		point2 = str(point2)[1:-1]
		return self.sendCommand('curve ' + point1 + ' ' + point2 + ' ' + str(speed))

	# Set Commands
	def setSpeed(self, speed):
		'''Sets speed given in cm/s. Range from 10 - 100 cm/s.'''
		return self.sendCommand('speed ' + str(speed))

	def rc(self, lr=0, fb=0, ud=0, yaw=0):
		'''
		Allows for 4 channel remote control type commands to be sent. 
		The limits for each input is -100 to +100 and the command has to only be
		sent once (i.e. no point is sending the same rc command repeatedly)
		'''
		command_string = 'rc ' + str(lr) + ' ' + str(fb) + ' ' + str(ud) \
				+ ' ' + str(yaw)
		try:
			# Only send rc command if it is different to the previous one
			if command_string != self.prev_rc_command:
				self.prev_rc_command = command_string
				return self.sendCommandNoWait(command_string)
		except AttributeError:
			# First run therefore declare the attribute and send command
			self.prev_rc_command = command_string
			return self.sendCommandNoWait(command_string)

	# Read Commands
	def getSpeed(self):
		'''Returns speed in cm/s.'''
		return self.sendCommand('speed?')

	def getBattery(self):
		'''Returns battery percentage.'''
		return self.sendCommand('battery?')

	def getTime(self):
		'''Returns flight time since turned on in seconds.'''
		return self.sendCommand('time?')

	def getHeight(self):
		'''
		Returns height in dm (decimeter) from starting point 
		This may be IMU based hence experiences drift. The return 
		value is rounded to the nearest decimeter.
		'''
		return self.sendCommand('height?')

	def getTemp(self):
		'''Returns temperature in Celcius.'''
		return self.sendCommand('temp?')

	def getAttitude(self):
		'''
		Returns IMU attitude of pitch, roll and yaw in degrees. 
		Yaw is zeroed when Tello is turned on.
		'''
		return self.sendCommand('attitude?')

	def getBaro(self):
		'''
		Returns barometer value (altitude) in metres.
		Seems to fluctuate a fair amount
		'''
		return self.sendCommand('baro?')

	def getAcceleration(self):
		'''
		Returns IMU acceleration in x, y and z directions in 
		0.001g's. So 1000 = 1g.
		'''
		return self.sendCommand('acceleration?')

	def getToF(self):
		'''
		Returns distance from the bottom mounted 'Time-of-Flight Camera'
		in mm. Minimum is 100 mm and anything below 300 mm defaults to 100
		mm. Seems fast and accurate.
		'''
		return self.sendCommand('tof?')

	def getSNR(self):
		'''Returns Signal to Noise Ratio of the WiFi link.'''
		return self.sendCommand('wifi?')


if __name__ == "__main__":
	drone = Tello1()
	drone.getBattery()
	drone.startVideoCapture()
	
	while True:
		frame = drone.readFrame()
		cv.imshow('Frame', frame)
		if cv.waitKey(1) & 0xFF == ord('q'):
			drone.shutdown()
			break
