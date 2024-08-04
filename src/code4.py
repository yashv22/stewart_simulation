#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math
import time

def send_sinusoidal_command():
    pub = rospy.Publisher('stewart/platform_pose', Twist, queue_size=10)
    rospy.init_node('stewart_command_sinusoidal', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    start_time = time.time()

    # PD controller previous errors
    roll_error_prev = 0
    pitch_error_prev = 0
    heave_error_prev = 0

    # Control gains
    Kp = 0.5  # Proportional gain
    Kd = 0.1  # Derivative gain

    # Sinusoidal parameters
    amplitude = 0.3  # Reduced amplitude for roll and pitch
    heave_amplitude = 0.5  # Reduced amplitude for heave
    frequency = 0.5  # Reduced frequency

    # Initialize last command values for filtering
    last_roll_command = 0
    last_pitch_command = 0
    last_heave_command = 0

    # Filtering factor for low-pass filter
    alpha = 0.5

    while not rospy.is_shutdown():
        current_time = time.time() - start_time

        # Desired sinusoidal values for roll, pitch, and heave
        desired_roll = amplitude * math.sin(frequency * current_time)
        desired_pitch = amplitude * math.sin(frequency * current_time + math.pi / 2)
        desired_heave = heave_amplitude * math.sin(frequency * current_time) + 0.4

        # Mock feedback (replace with actual feedback from sensors)
        feedback_roll = last_roll_command
        feedback_pitch = last_pitch_command
        feedback_heave = last_heave_command

        # Calculate errors
        roll_error = desired_roll - feedback_roll
        pitch_error = desired_pitch - feedback_pitch
        heave_error = desired_heave - feedback_heave

        # PD control for roll, pitch, and heave
        roll_command = desired_roll + Kp * roll_error + Kd * (roll_error - roll_error_prev)
        pitch_command = desired_pitch + Kp * pitch_error + Kd * (pitch_error - pitch_error_prev)
        heave_command = desired_heave + Kp * heave_error + Kd * (heave_error - heave_error_prev)

        # Update previous errors
        roll_error_prev = roll_error
        pitch_error_prev = pitch_error
        heave_error_prev = heave_error

        # Apply low-pass filter to smooth the commands
        filtered_roll_command = alpha * roll_command + (1 - alpha) * last_roll_command
        filtered_pitch_command = alpha * pitch_command + (1 - alpha) * last_pitch_command
        filtered_heave_command = alpha * heave_command + (1 - alpha) * last_heave_command

        # Update last commands
        last_roll_command = filtered_roll_command
        last_pitch_command = filtered_pitch_command
        last_heave_command = filtered_heave_command

        # Prepare the Twist message
        command = Twist()
        command.angular.x = filtered_roll_command
        command.angular.y = filtered_pitch_command
        command.linear.z = filtered_heave_command

        rospy.loginfo("Sending roll: %s, pitch: %s, heave: %s", filtered_roll_command, filtered_pitch_command, filtered_heave_command)
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_sinusoidal_command()
    except rospy.ROSInterruptException:
        pass

