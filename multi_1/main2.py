
import time
import numpy as np
import logging

from tello2 import Tello2
from controller2 import runWaypoint
from position2 import (startTracking, trueState, waypointGeneration,
        waypointUpdate)


# Initialise Tello
tello = Tello2()
tello.getBattery()
# tello.startStateCapture()
tello.rc()	# reset controls to zeros

fly = True
if fly:
    tello.takeoff()
    # tello.land()
    time.sleep(10)
    tello.move('up', 80)
    # tello.rc(0,10,0,40)
    time.sleep(5)
    # tello.land()
    # time.sleep(5)


# Setup connection with optitrack and the waypoints
optitrack = True
if optitrack:
    
    threads = []

    # # Ask for Optitrack body1 number
    # body_id_drone1 = input("Enter Body1 number for optitrack: ")
    #
    # # Ask for Optitrack body2 number
    # body_id_drone2 = input("Enter Body2 number for optitrack: ")
    
    streamingClient = startTracking(2, 3)	# Get rigid body ID from Motive (4, 3)
    waypoint = waypointGeneration(streamingClient)
    # print('Way point successfully generated')

# Initialise variables and flags
launch_controller = True
time_initialise = time.time()

# Ready to run?
go = input("Ready to fly Tello2 ?")

# Lets run the control loop (Open)
try:
    while True:
        # Reset variables
        mode = None
        true_state = None
        estimated_state = None
        distance = None

        # CONTROLLER
        if launch_controller:
        # Update current waypoint and get vector to current waypoint


            # This is running only if Opti-Track is connected
            if optitrack:

                # Obtain Body1 ID
                id_tello = trueState(streamingClient)[0][0]

                # Obtain Body1 3D Position Data
                pos_tello = trueState(streamingClient)[1][0,:]

                # Obtain Body1 3D Rotation Data
                rot_tello = trueState(streamingClient)[2][:,0].T

                # Obtain Body1 3D Euler Angles Data
                euler_tello = trueState(streamingClient)[3][0,:]

                ## Store all above information into true state
                true_state = np.array([id_tello, pos_tello, rot_tello, euler_tello])
                # true_position_gate = trueState(streamingClient)[1][1,:]

                relative_vector = waypointUpdate(streamingClient, true_state, waypoint)
                # print('Waypoint Updated, Go to next waypoint!!!')
                # print('relative vector generated')

            end_time = time.time()
            try:
                    dt = end_time - start_time
            except NameError:
                    dt = 0.005	# Approximate dt for first iteration

            # Run Waypoint tracking if optitrack is running
            if optitrack:
                # print('Now lets run the runWaypoint')
                runWaypoint(true_state, streamingClient, relative_vector, dt, tello)
                # print('runWaypoint Successfull')


        start_time = time.time()

    time.sleep(5)
    tello.land()
    # tello.shutdown()

except Exception:
    tello.rc()
    print("Main Controller Loop Crashed...")
    raise

