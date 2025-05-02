#!/usr/bin/env python3

import sys
import grpc

from triomnidrive_control.proto import rpi_motor_pb2
from triomnidrive_control.proto import rpi_motor_pb2_grpc

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class GrpcClient(Node):
    def __init__(self):
        super().__init__('grpc_client_node')

        self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)

        self.u = 0.0    # Linear 'x' vel. in omni-directional base
        self.v = 0.0    # Linear 'y' vel. in omni-directional base
        self.w = 0.0    # Linear height vel. in elevator

        self.q = 0.0    # Angular pitch setpoint of camera
        self.r = 0.0    # Angular yaw vel. in omni-directional base

        self.port = 50201
        self.channel = grpc.insecure_channel('192.168.2.103:'+str(self.port))  
        self.client = rpi_motor_pb2_grpc.RPIMotorStub(self.channel)

        self.req = rpi_motor_pb2.StateRequest()
        
    def listener_callback(self, msg):
        self.u = msg.linear.x
        self.v = msg.linear.y
        self.w = msg.linear.z
        
        self.q = msg.angular.y
        self.r = msg.angular.z

        self.req.vel_x = self.u
        self.req.vel_y = self.v
        self.req.vel_yaw = self.r

        self.req.vel_heave = self.w
        self.req.pitch = self.q
        self.req.leds = 0.

        res = self.client.SetState(self.req)
        self.get_logger().info("New velocity input: %f, %f, %f" % (self.u, self.v, self.r))
  
def main(args=None):
    rclpy.init(args=args)

    try:
        node = GrpcClient()
    except Exception as e:
        print(f"[FATAL] GrpcClient failed to initialize: {e}.", file=sys.stderr)
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted with Ctrl+C.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()