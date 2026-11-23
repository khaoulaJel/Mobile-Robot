#!/usr/bin/env python3
"""Do not spawn controllers until the laptop's robot_description has initialized hardware."""
import time
import rclpy
from controller_manager_msgs.srv import ListHardwareComponents
from lifecycle_msgs.msg import State


def main():
    rclpy.init();node=rclpy.create_node('wait_for_robot_hardware')
    client=node.create_client(ListHardwareComponents,'/controller_manager/list_hardware_components')
    node.get_logger().info('Waiting for the laptop robot_description and active Arduino hardware.')
    try:
        while rclpy.ok():
            if not client.wait_for_service(timeout_sec=1.0):continue
            future=client.call_async(ListHardwareComponents.Request())
            rclpy.spin_until_future_complete(node,future,timeout_sec=2.0)
            if future.done() and future.result() is not None:
                if any(c.name=='RealRobot' and c.state.id==State.PRIMARY_STATE_ACTIVE
                       for c in future.result().component):
                    node.get_logger().info('Arduino hardware ready; starting controllers.')
                    return 0
            if not future.done():client.remove_pending_request(future)
            time.sleep(0.2)
    except KeyboardInterrupt:
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 1


if __name__=='__main__':
    raise SystemExit(main())
