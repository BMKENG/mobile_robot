import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class WaypointSaver(Node):

    def __init__(self):
        super().__init__('waypoint_saver')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback,
            10)
        self.waypoint = 0.0

    def listener_callback(self, msg):
        self.waypoint = msg.pose.pose
        print(self.waypoint)

    def setWaypoint(self):
        if self.waypoint is not None:
            return self.waypoint
        else:
            self.get_logger().warn("아직 유효한 waypoint가 저장되지 않았습니다.")
            return None
        
def main(args=None):
    rclpy.init(args=args)

    waypoint_saver = WaypointSaver()

    rclpy.spin(waypoint_saver)  # 노드를 실행하고 토픽을 듣도록 합니다.

    waypoint_saver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
