#! /usr/bin/python3

import threading
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int64MultiArray

kp = 0
ki = 0
kd = 0
setpoint = 0


class MotorFeedbackListener(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Int64MultiArray, '/motor/feedback', self.callback, 10
        )
        self.left_pub = self.create_publisher(
            Int32, '/motor/left_cmd', 10
        )
        self.right_pub = self.create_publisher(
            Int32, '/motor/right_cmd', 10
        )
        self.timer = self.create_timer(
            0.2, self.cmd_callback
        )
        self.last_time = time.time()
        self.last_left_counts = 0
        self.last_right_counts = 0
        self.total_counts_per_revolution = 151
        self.left_motor_rpm = 0
        self.right_motor_rpm = 0
        self.cmd_left_motor_rpm = 0
        self.cmd_right_motor_rpm = 0
        self.left_motor_cmd_msg = Int32()
        self.right_motor_cmd_msg = Int32()

    def callback(self, msg):
        if len(msg.data) >= 2:
            current_time = time.time()
            elapsed_time = current_time - self.last_time

            left_counts = msg.data[0]
            right_counts = msg.data[1]

            left_counts_delta = left_counts - self.last_left_counts
            right_counts_delta = right_counts - self.last_right_counts

            self.left_motor_rpm = self.calculate_rpm(left_counts_delta, elapsed_time)
            self.right_motor_rpm = self.calculate_rpm(right_counts_delta, elapsed_time)

            self.last_time = current_time
            self.last_left_counts = left_counts
            self.last_right_counts = right_counts

            self.get_logger().info(
                f'Left: {round(self.left_motor_rpm, 1)}, Right: {round(self.right_motor_rpm, 1)}'
            )

    def cmd_callback(self):
        left = 0
        right = 0
        if self.cmd_left_motor_rpm > 255:
            left = 255
        elif self.cmd_left_motor_rpm < -255:
            left = -255
        else:
            left = int(self.cmd_left_motor_rpm)

        if self.cmd_right_motor_rpm > 255:
            right = 255
        elif self.cmd_right_motor_rpm < -255:
            right = -255
        else:
            right = int(self.cmd_right_motor_rpm)
        self.left_motor_cmd_msg.data = left
        self.right_motor_cmd_msg.data = right
        self.left_pub.publish(self.left_motor_cmd_msg)
        self.right_pub.publish(self.right_motor_cmd_msg)

    def get_left_motor_rpm(self):
        return self.left_motor_rpm

    def get_right_motor_rpm(self):
        return self.right_motor_rpm

    def set_left_motor_rpm(self, cmd_pwm):
        self.cmd_left_motor_rpm = cmd_pwm

    def set_right_motor_rpm(self, cmd_pwm):
        self.cmd_right_motor_rpm = cmd_pwm

    def calculate_rpm(self, counts_delta, elapsed_time):
        counts_per_revolution = self.total_counts_per_revolution
        rpm = (counts_delta / counts_per_revolution) / (elapsed_time / 60.0)
        return rpm


class PIDController:

    def __init__(self, kp, ki, kd, setpoint, sample_time=10, proportional_on_error=True):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.sample_time = sample_time
        self.proportional_on_error = proportional_on_error

        self.prev_input = 0
        self.output_sum = 0
        self.last_time = time.time()

        self.output_min, self.output_max = -255, 255

    def set_const(self, Kp, Ki, Kd, Setpoint):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.setpoint = Setpoint

    def compute(self, feedback):
        current_time = time.time()

        input_val = feedback
        error = self.setpoint - input_val
        d_input = (input_val - self.prev_input)

        self.output_sum += self.ki * error

        # Add Proportional on Measurement if P_ON_E is not specified
        if not self.proportional_on_error:
            self.output_sum -= self.kp * d_input

        if self.output_sum > self.output_max:
            self.output_sum = self.output_max
        elif self.output_sum < self.output_min:
            self.output_sum = self.output_min

        # Add Proportional on Error if P_ON_E is specified
        output = self.kp * error if self.proportional_on_error else 0

        # Compute the rest of PID output
        output += self.output_sum - self.kd * d_input

        if output > self.output_max:
            output = self.output_max
        elif output < self.output_min:
            output = self.output_min

        # Update variables for the next iteration
        self.prev_input = input_val
        self.last_time = current_time

        return output


class PIDGUI:

    def __init__(self, master):
        self.master = master
        self.master.title('PID Controller GUI')

        self.default_kp = 0.71
        self.default_ki = 0.352
        self.default_kd = 0.0857
        self.default_setpoint = 30.0

        self.kp_label = tk.Label(master, text='Enter Kp:')
        self.kp_label.pack()

        self.kp_entry = tk.Entry(master)
        self.kp_entry.insert(tk.END, str(self.default_kp))
        self.kp_entry.pack()

        self.ki_label = tk.Label(master, text='Enter Ki:')
        self.ki_label.pack()

        self.ki_entry = tk.Entry(master)
        self.ki_entry.insert(tk.END, str(self.default_ki))
        self.ki_entry.pack()

        self.kd_label = tk.Label(master, text='Enter Kd:')
        self.kd_label.pack()

        self.kd_entry = tk.Entry(master)
        self.kd_entry.insert(tk.END, str(self.default_kd))
        self.kd_entry.pack()

        self.setpoint_label = tk.Label(master, text='Enter Setpoint:')
        self.setpoint_label.pack()

        self.setpoint_entry = tk.Entry(master)
        self.setpoint_entry.insert(tk.END, str(self.default_setpoint))
        self.setpoint_entry.pack()

        self.set_button = tk.Button(master, text='Set', command=self.print_values)
        self.set_button.pack()

    def print_values(self):
        try:
            kp = float(self.kp_entry.get())
            ki = float(self.ki_entry.get())
            kd = float(self.kd_entry.get())
            setpoint = float(self.setpoint_entry.get())
            left_pid.set_const(kp, ki, kd, setpoint)
            right_pid.set_const(kp, ki, kd, setpoint)

            print(f'Kp: {kp}, Ki: {ki}, Kd: {kd}, Setpoint: {setpoint}')
        except ValueError:
            print('Invalid input. Please enter numeric values.')


left_pid = None
right_pid = None
kp = 0
ki = 0
kd = 0
setpoint = 0


def main(args=None):
    global left_pid, right_pid
    rclpy.init(args=args)
    motor_feedback_listener = MotorFeedbackListener()
    left_pid = PIDController(1, 0, 0, 0)
    right_pid = PIDController(1, 0, 0, 0)
    gui = threading.Thread(target=run_gui)
    gui.start()
    while True:
        left = left_pid.compute(motor_feedback_listener.get_left_motor_rpm())
        right = right_pid.compute(motor_feedback_listener.get_right_motor_rpm())
        motor_feedback_listener.set_left_motor_rpm(left)
        motor_feedback_listener.set_right_motor_rpm(right)
        # print(kp, ki, kd, setpoint, left, right)
        rclpy.spin_once(motor_feedback_listener)
    rclpy.spin(motor_feedback_listener)
    motor_feedback_listener.destroy_node()
    rclpy.shutdown()


def run_gui():
    root = tk.Tk()
    PIDGUI(root)
    root.mainloop()


if __name__ == '__main__':
    main()
