from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Devices.DCMotor import *
import traceback
import time
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist


# Max velocities for scaling
MAX_LINEAR_VELOCITY = 3.0 # m/s
MAX_ANGULAR_VELOCITY = 1.0 # rad/s

def setup_motor(number,channel):
	motor = DCMotor()
	motor.setDeviceSerialNumber(number)
	motor.setChannel(channel)
	motor.openWaitForAttachment(5000)
	return motor


dcMotor1 = setup_motor(487541, 0)
dcMotor2 = setup_motor(487541, 1)
dcMotor3 = setup_motor(487736, 0)
dcMotor4 = setup_motor(487736, 1)


class CmdVelSubscriber(Node):
	def __init__(self):
		super().__init__('cmd_vel_subscriber')
		self.subscription = self.create_subscription(
			Twist,
			'/cmd_vel',
			self.cmd_vel_callback,
			10)
		self.subscription

	def cmd_vel_callback(self, msg):
		vx = msg.linear.x
		vy = msg.linear.y
		omega = msg.angular.z

		vx = max(min(vx,MAX_LINEAR_VELOCITY), -MAX_LINEAR_VELOCITY)
		vy = max(min(vy, MAX_LINEAR_VELOCITY), -MAX_LINEAR_VELOCITY)
		omega = max(min(omega, MAX_ANGULAR_VELOCITY), -MAX_ANGULAR_VELOCITY)

		motor_speeds = mecanum_kinematic_model(vx, vy, omega)

		max_speed = max(abs(speed) for speed in motor_speeds)
		if max_speed > 1.0:
			motor_speeds = [speed/ max_speed for speed in motor_speeds]
		
		# Send motor speeds in the vorrect order: backleft, backright, frontright, frontleft
		set_motor_speeds(motor_speeds)


def mecanum_kinematic_model(vx,vy,omega):
	Ly = 1 # Distance between front and back wheels in meters
	Lx = 1 # Distance between left and right wheels in meters
	R = 1 # Wheels radius in meters

	mat = np.matric([
		[1,-1,-(Ly + Lx)],
		[1, 1, (Ly + Lx)],
		[1, 1,-(Ly + Lx)],
		[1,-1, (Ly + Lx)]
	])

	cmd_vel = np.matrix([vx, vy, omega]).T
	wheel_speeds = (1/R) * (mat * cmd_vel).A1

	return wheel_speeds.tolist()

def set_motor_speeds(speeds):
	dcMotor1.setTargetVelocity(speeds[3])
	dcMotor2.setTargetVelocity(-speeds[2])
	dcMotor3.setTargetVelocity(-speeds[0])
	dcMotor4.setTargetVelocity(speeds[1])


def main():
	rclpy.init(args=args)

	cmd_vel_subscriber = CmdVelSubscriber()

	rclpy.spin(cmd_vel_subscriber)
    #Declare any necessary variables here

	#Close your Phidgets once the program is done.
	dcMotor1.close()
	dcMotor2.close()
	dcMotor3.close()
	dcMotor4.close()

	cmd_vel_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
    main()
