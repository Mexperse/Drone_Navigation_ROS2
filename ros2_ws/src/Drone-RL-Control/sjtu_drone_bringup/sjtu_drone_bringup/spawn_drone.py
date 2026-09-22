#!/usr/bin/env python3
import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, Point, Quaternion
import math


def yaw_to_quaternion(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('spawn_drone')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    content = sys.argv[1]
    namespace = sys.argv[2]

    # Optional pose args: x y z yaw (all default to 0.0 if not provided)
    x = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    y = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    z = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
    yaw = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0

    req = SpawnEntity.Request()
    req.name = namespace
    req.xml = content
    req.robot_namespace = namespace
    req.reference_frame = "world"
    req.initial_pose = Pose(
        position=Point(x=x, y=y, z=z),
        orientation=yaw_to_quaternion(yaw)
    )

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            'Result ' + str(future.result().success) + " " + future.result().status_message)
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()