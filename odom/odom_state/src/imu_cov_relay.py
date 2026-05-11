#!/usr/bin/env python3
"""
IMU Covariance Relay Node

Subscribes to an input Imu topic, optionally replaces covariance diagonals with
configured values, and republishes to an output topic.

Use this when a driver publishes placeholder/default covariances but you want
to inject measured values for EKF fusion.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuCovRelay(Node):
	def __init__(self):
		super().__init__('imu_cov_relay')

		self.declare_parameter('input_topic', '/imu/data_raw')
		self.declare_parameter('output_topic', '/imu/data')
		self.declare_parameter('override_covariance', True)
		self.declare_parameter('force_unknown_fields', False)

		self.declare_parameter('ang_vel_stddev', [0.00896889, 0.00649372, 0.01154264])
		self.declare_parameter('lin_acc_stddev', [0.00071867, 0.00115473, 0.00050693])

		self.lin_acc_stddev = self.get_parameter('lin_acc_stddev').value
		self.ang_vel_stddev = self.get_parameter('ang_vel_stddev').value
		self.input_topic = self.get_parameter('input_topic').value
		self.output_topic = self.get_parameter('output_topic').value
		self.override_covariance = self.get_parameter('override_covariance').value
		self.force_unknown_fields = self.get_parameter('force_unknown_fields').value

		# self.orientation_var = [
		# 	float(self.get_parameter('orientation_stddev_roll').value) ** 2,
		# 	float(self.get_parameter('orientation_stddev_pitch').value) ** 2,
		# 	float(self.get_parameter('orientation_stddev_yaw').value) ** 2,
		# ]
		self.angular_velocity_var = [
			float(self.ang_vel_stddev[0]) ** 2,
			float(self.ang_vel_stddev[1]) ** 2,
			float(self.ang_vel_stddev[2]) ** 2,
		]
		self.linear_acceleration_var = [
			float(self.lin_acc_stddev[0]) ** 2,
			float(self.lin_acc_stddev[1]) ** 2,
			float(self.lin_acc_stddev[2]) ** 2,
		]

		self.sub = self.create_subscription(Imu, self.input_topic, self.callback, 10)
		self.pub = self.create_publisher(Imu, self.output_topic, 10)

		if self.override_covariance:
			self.get_logger().info(
				'IMU cov relay: overriding covariances '
				f'({self.input_topic} -> {self.output_topic}), '
				f'force_unknown_fields={self.force_unknown_fields}'
			)
			self.get_logger().info(
				'Injected variances '
				# f'orientation={self.orientation_var}, '
				f'ang_vel={self.angular_velocity_var}, '
				f'lin_acc={self.linear_acceleration_var}'
			)
		else:
			self.get_logger().info(
				f'IMU cov relay: pass-through mode ({self.input_topic} -> {self.output_topic})'
			)

	@staticmethod
	def _diag3(diag_values):
		cov = [0.0] * 9
		cov[0] = diag_values[0]
		cov[4] = diag_values[1]
		cov[8] = diag_values[2]
		return cov

	def callback(self, msg: Imu):
		if self.override_covariance:
			# Respect unknown-field sentinel unless explicitly forced.
			# if self.force_unknown_fields or abs(msg.orientation_covariance[0] + 1.0) > 1e-9:
			# 	msg.orientation_covariance = self._diag3(self.orientation_var)

			if self.force_unknown_fields or abs(msg.angular_velocity_covariance[0] + 1.0) > 1e-9:
				msg.angular_velocity_covariance = self._diag3(self.angular_velocity_var)

			if self.force_unknown_fields or abs(msg.linear_acceleration_covariance[0] + 1.0) > 1e-9:
				msg.linear_acceleration_covariance = self._diag3(self.linear_acceleration_var)

		self.pub.publish(msg)


def main(args=None):
	try:
		rclpy.init(args=args)
		node = ImuCovRelay()
		try:
			rclpy.spin(node)
		except KeyboardInterrupt:
			node.get_logger().info('Shutting down imu_cov_relay node...')
		except Exception as exc:
			node.get_logger().error(f'Unexpected error: {exc}')
		finally:
			node.destroy_node()
	except Exception as exc:
		print(f'Failed to initialize IMU Covariance Relay: {exc}')
	finally:
		if rclpy.ok():
			rclpy.shutdown()


if __name__ == '__main__':
	main()
