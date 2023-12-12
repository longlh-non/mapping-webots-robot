"""help controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
import numpy as np
import matplotlib.pyplot as plt


# This is written as a function for convenience
def get_yaw(iu):
    v = iu.getRollPitchYaw()
    yaw = round(math.degrees(v[2]))
    # The InertialUnit gives us a reading based on how the robot is oriented with
    # respect to the X axis of the world: EAST 0°, NORTH 90°, WEST 180°, SOUTH -90°.
    # This operation converts these values to a normal, positive circumfrence.
    if yaw < 0:
        yaw += 360
    return yaw


def start_mapping(robot):
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    MAX_SPEED = 6.28

    X=0
    theta=0

    xw=0
    yw=0.0277
    theta=1.57 #90 degree in radian
    # Threshol: maximum allowed error
    angle_treshold = 1
    distance_threshold = 0.002
    # OBJECTIVES
    degrees = 90  # Desired amount of rotation
    distance = 0.2  # Desired amount of linear translation

    angles = np.linspace(3.1415, -3.1415, 360)

    # Sensors

    iu = robot.getDevice('inertial unit')
    iu.enable(timestep)
    gps = robot.getDevice('gps')
    gps.enable(timestep)

    lidar = robot.getDevice('LDS-01')
    lidar.enable(timestep)
    lidar.enablePointCloud()

    # Some devices, such as the InertialUnit, need some time to "warm up"
    robot.step(1000)

    # Actuators
    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)

    # Enable Distance sensor
    prox_sensors = []
    for i in range(8):
        prox_sensors.append(robot.getDevice(f"ps{i}"))
        prox_sensors[i].enable(timestep)

    # Calculate the parameters of the simulation

    print("Turning {0} degrees".format(degrees))
    starting_yaw = get_yaw(iu)
    print("Start: " + str(starting_yaw))
    target = (starting_yaw - degrees) % 360
    print("Target: " + str(target))

    # Calculate the desired ending coordinate
    destination_coordinate = [
        distance * math.cos(math.radians(target)),
        distance * math.sin(math.radians(target))
    ]

    print("Final coordinate: {0}".format(destination_coordinate))

    # Start executing
    while robot.step(timestep) != -1:

        for i, sens in enumerate(prox_sensors):
            print(f"ps{i}: {sens.getValue()}")

        # CHeck sensor data
        # front_wall = prox_sensors[7].getValue() > 70 or prox_sensors[0].getValue() > 70
        # left_wall = prox_sensors[5].getValue() > 80
        # left_corner = prox_sensors[6].getValue() > 80
        # right_wall = prox_sensors[2].getValue() > 80
        # right_corner = prox_sensors[1].getValue() > 80
        # no_wall_at_all = prox_sensors[1].getValue() < 80 and prox_sensors[2].getValue() < 80 and left_wall == False and left_corner == False and front_wall == False
        ranges = lidar.getRangeImage()

        # # Enter here functions to send actuator commands, like:
        # #  motor.setPosition(10.0)

        leftSpeed = MAX_SPEED
        rightSpeed = MAX_SPEED

        # print("No wall found")
        # if no_wall_at_all:
        #     print("No wall found")
        # else:
        #     if front_wall:
        #         print("Turn right")
        #         leftSpeed = MAX_SPEED
        #         rightSpeed = -MAX_SPEED
        #
        #     else:
        #         if left_wall:
        #             print("Wall on the left")
        #             leftSpeed = MAX_SPEED
        #             rightSpeed = -MAX_SPEED
        #         else:
        #             print("Turn left")
        #             leftSpeed = -MAX_SPEED / 8
        #             rightSpeed = MAX_SPEED
        #
        #         if left_corner:
        #             leftSpeed = MAX_SPEED
        #             rightSpeed = MAX_SPEED / 8

                # if right_wall:
                #     print("Wall on the right")
                #     leftSpeed = -MAX_SPEED
                #     rightSpeed = MAX_SPEED
                # else:
                #     print("Turn right")
                #     leftSpeed = -MAX_SPEED
                #     rightSpeed = MAX_SPEED / 8
                #
                # if right_corner:
                #     leftSpeed = MAX_SPEED / 8
                #     rightSpeed = MAX_SPEED

        # First rotate
        current_yaw = get_yaw(iu)
        if abs(target - current_yaw) > angle_treshold:
            leftSpeed = 0.3 * MAX_SPEED
            rightSpeed = -0.3 * MAX_SPEED
        else:
            # When rotation is complete, go straight
            current_coordinate = gps.getValues()
            distance_to_target_x = abs(current_coordinate[0] - destination_coordinate[0])
            distance_to_target_y = abs(current_coordinate[1] - destination_coordinate[1])
            if distance_to_target_x < distance_threshold and distance_to_target_y < distance_threshold:
                leftSpeed = 0
                rightSpeed = 0
            else:
                leftSpeed = MAX_SPEED
                rightSpeed = MAX_SPEED


        x_r, y_r = [], []
        x_w, y_w = [], []
        for i, angle in enumerate(angles):
            x_i = ranges[i]*np.cos(angle)
            y_i = ranges[i]*np.sin(angle)
            x_r.append(x_i)
            y_r.append(y_i)
            x_w.append(np.cos(theta)*x_i + np.cos(theta+1.57)*y_i +xw)
            y_w.append(np.cos(theta-1.57)*x_i + np.cos(theta)*y_i +yw)

        plt.ion()
        plt.plot(x_w, y_w, '.')
        plt.pause(0.01)
        plt.show()


        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)

        factor = 0.0186
        deltaX = (leftSpeed * factor + rightSpeed * factor) / 2 * timestep / 1000
        X += deltaX
        theta += (rightSpeed * factor - leftSpeed * factor) / 0.052 * timestep / 1000
        xw += np.cos(theta) * deltaX
        yw += np.sin(theta) * deltaX


if __name__ == "__main__":
    # Create robot instance.
    new_robot = Robot()
    start_mapping(new_robot)
