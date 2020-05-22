
import time
import numpy as np

from tello1 import Tello1
from controller1 import runWaypoint
from position1 import (connectOptitrack, droneState, waypointGeneration,
        waypointUpdate)


# Initialise Tello
tello = Tello1()
tello.getBattery()
tello.rc()

fly = True
if fly:
    tello.takeoff()
    time.sleep(10)
    tello.move('up', 200)
    time.sleep(5)

# Setup connection with optitrack and the waypoints
optitrack = True
if optitrack:
    
    threads = []

    # # Ask for Optitrack body1 number
    # body_id_drone1 = input("Enter Body1 number for optitrack: ")
    #
    # # Ask for Optitrack body2 number
    # body_id_drone2 = input("Enter Body2 number for optitrack: ")
    
    streamingClient = connectOptitrack(1, 3)	# Get rigid body ID from Motive (4, 3)
    waypoint = waypointGeneration(streamingClient)

# Initialise variables
launch_controller = True
time_initialise = time.time()

# Ready to run?
go = input("Ready to fly Tello1 ?")

# Lets run the control loop
try:
    while True:
        # Reset variables
        mode = None
        true_state = None
        estimated_state = None
        distance = None

        # Initiate the Position CONTROLLER
        if launch_controller:

            # This is running only if Opti-Track is connected
            if optitrack:

                # Obtain Body1 ID
                id_tello = droneState(streamingClient)[0][0]

                # Obtain Body1 3D Position Data
                pos_tello = droneState(streamingClient)[1][0,:]

                # Obtain Body1 3D Rotation Data
                rot_tello = droneState(streamingClient)[2][:,0].T

                # Obtain Body1 3D Euler Angles Data
                euler_tello = droneState(streamingClient)[3][0,:]

                ## Store all above information into true state
                true_state = np.array([id_tello, pos_tello, rot_tello, euler_tello])
                # true_position_gate = droneState(streamingClient)[1][1,:]

                relative_vector = waypointUpdate(streamingClient, true_state, waypoint)


            end_time = time.time()
            try:
                    dt = end_time - start_time
            except NameError:
                    dt = 0.005	# Approximate dt for first iteration

            # Run Waypoint tracking if optitrack is running
            if optitrack:
                runWaypoint(true_state, streamingClient, relative_vector, dt, tello)

        start_time = time.time()

    time.sleep(5)
    tello.land()

except Exception:
    tello.rc()
    tello.land()
    print("Main Controller Loop Crashed...")
    raise

