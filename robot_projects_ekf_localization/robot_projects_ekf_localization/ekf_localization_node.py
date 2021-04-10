import rclpy
from sensor_msgs.msg import Imu, PointCloud2
from rclpy.node import Node
from transforms3d.euler import quat2euler
import pcl_msgs


class EkfNodeWrapper(Node):
    def __init__(self):
        super().__init__("ekf_localization")
        self.imu_sub = self.create_subscription(
            Imu,
            "imu/reading",
            self.imu_callback,
            10
        )
        self.point_sun = self.create_subscription(
            PointCloud2,
            "/pt_sensor/reading",
            self.point_callback,
            10
        )

    def imu_callback(self, msg):
        o_quat = [
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ]
        _, _, yaw = quat2euler(o_quat)

    def point_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = EkfNodeWrapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()