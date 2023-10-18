#!/usr/bin/env python3

# This script is used to set the parameters of the device when ROS is enabled on the device.
# This node is used to publish the parameters to the given topics

import rospy
from ..lib.ROSHandler.src.ros_lib.std_msgs import Float32MultiArray

motor_index = 0  # 0: Finger, 1: Palm, 2: Both

paramaters_finger = [4,         # Wave Type (0:CONSTANT, 1:SQUARE, 2:SAWTOOTH, 3:TRIANGULAR, 4:SINUSOIDAL, 5:TRAPEZOIDAL)
                     4,         # Wave Frequency
                     500000,      # Wave Amplitude
                     0,      # Wave Offset
                     15,          # Kp Gain
                     0.001,           # Ki Gain
                     5]           # Kd Gain

paramaters_palm = [0,           # Wave Type (0:CONSTANT, 1:SQUARE, 2:SAWTOOTH, 3:TRIANGULAR, 4:SINUSOIDAL)
                   2,         # Wave Frequency
                   0,      # Wave Amplitude
                   0,      # Wave Offset
                   13,          # Kp Gain
                   0,           # Ki Gain
                   3]           # Kd Gain

params = [paramaters_finger, paramaters_palm]
allowable_motor_index = [0, 1, 2]


class ParamPublisher:
    def __init__(self):
        self.publishers = []
        self.publishers.append(rospy.Publisher(
            "/set_waveform_type", Float32MultiArray, queue_size=10))
        self.publishers.append(rospy.Publisher(
            "/set_waveform_frequency", Float32MultiArray, queue_size=10))
        self.publishers.append(rospy.Publisher(
            "/set_waveform_amplitude", Float32MultiArray, queue_size=10))
        self.publishers.append(rospy.Publisher(
            "/set_waveform_offset", Float32MultiArray, queue_size=10))
        self.publishers.append(rospy.Publisher(
            "/set_Kp", Float32MultiArray, queue_size=10))
        self.publishers.append(rospy.Publisher(
            "/set_Ki", Float32MultiArray, queue_size=10))
        self.publishers.append(rospy.Publisher(
            "/set_Kd", Float32MultiArray, queue_size=10))

        self.send_data = [True] * len(self.publishers)


def main():
    if motor_index not in allowable_motor_index:
        exit(
            f"Motor index can only be one of the following values: {allowable_motor_index}\nExiting.")
    try:
        rospy.init_node('param_configurator')
        parameter_publisher = ParamPublisher()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            for idx, publisher in enumerate(parameter_publisher.publishers):
                if not parameter_publisher.send_data[idx]:
                    continue
                connections = publisher.get_num_connections()
                if connections > 0:
                    if motor_index == 0:
                        arr = [motor_index, params[motor_index][idx]]
                        data_to_send = Float32MultiArray(data=arr)
                        publisher.publish(data_to_send)
                    elif motor_index == 1:
                        arr = [motor_index, params[motor_index][idx]]
                        data_to_send = Float32MultiArray(data=arr)
                        publisher.publish(data_to_send)
                    elif motor_index == 2:
                        for i in range(motor_index):
                            arr = [i, params[i][idx]]
                            data_to_send = Float32MultiArray(data=arr)
                            publisher.publish(data_to_send)
                            rospy.sleep(0.1)
                    parameter_publisher.send_data[idx] = False

            if not any(parameter_publisher.send_data):
                break
            rate.sleep()
        print("Done.")

    except rospy.ROSInterruptException as e:
        raise e


if __name__ == "__main__":
    main()
