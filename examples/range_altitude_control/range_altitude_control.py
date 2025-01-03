#!/usr/bin/env python

"""

range_altitude_controller.py: (Copter Only)

This example shows a PID controller regulating the altitude of Copter using a downward-facing rangefinder sensor.

Caution: A lot of unexpected behaviors may occur in GUIDED_NOGPS mode.
        Always watch the drone movement, and make sure that you are in dangerless environment.
        Land the drone as soon as possible when it shows any unexpected behavior.

Tested in Python 3.12.3

"""
import collections
import collections.abc

from simple_pid import PID

# The collections module has been reorganized in Python 3.12 and the abstract base
# classes have been moved to the collections.abc module. This line is necessary to
# fix a bug in importing the MutableMapping class in `dronekit`.

collections.MutableMapping = collections.abc.MutableMapping
from dronekit import connect, VehicleMode
from pymavlink import mavutil # Needed for command message definitions
import time
import math

import matplotlib.pyplot as plt

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect',
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

HOVER_THRUST = 0.4

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, timeout=80)

PID_SAMPLE_RATE = 50 # [Hz]

def altitude_controller(target_altitude : float):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """

    print("Arming motors")
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)

    print("Taking off!")
    pid_controller = PID(0.10, 0.05, 0.04, setpoint=target_altitude, sample_time=1/PID_SAMPLE_RATE, output_limits=(0, 1))

    range_zeroing_offset = vehicle.rangefinder.distance

    error_data = []
    try:
        while True:
            # Calculate the altitude error using the rangefinder sensor
            current_range = vehicle.rangefinder.distance
            altitude = current_range - range_zeroing_offset
            error = pid_controller.setpoint - altitude
            error_data.append(error)

            # Compute and set the thrust value from the PID controller
            thrust = pid_controller(current_range - range_zeroing_offset)
            print(f"Thrust: {thrust}\n Error: {error}\n")
            set_attitude(thrust=thrust)
            time.sleep(1/PID_SAMPLE_RATE)
    except KeyboardInterrupt:
        # Land the drone when the user interrupts the program
        print("Emergency Landing...")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        print("Closing vehicle object")
        vehicle.close()

        # Save the error data plot
        time_data = [i / PID_SAMPLE_RATE for i in range(len(error_data))]
        plt.plot(time_data, error_data)
        plt.xlabel('Time (s)')
        plt.ylabel('Error [m]')
        plt.title('Altitude Control Error')
        plt.savefig('altitude_control_error.png')


def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        time.sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

# Take off in GUIDED_NOGPS mode.
print("Hold position ")
altitude_controller(1)

print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
time.sleep(1)

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed")
