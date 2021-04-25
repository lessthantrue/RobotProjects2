import rclpy
from sensor_msgs.msg import Imu, PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from transforms3d.euler import quat2euler
import pcl_msgs
from robot_projects_ekf_localization import point_cloud2, ekf
import numpy as np

class EkfNodeWrapper(Node):
    def __init__(self):
        super().__init__("ekf_localization")
        self.imu_sub = self.create_subscription(
            Imu,
            "imu/reading",
            self.imu_callback,
            10
        )
        self.point_sub = self.create_subscription(
            PointCloud2,
            "/pt_sensor/reading",
            self.point_callback,
            10
        )
        self.est_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "pose_estimate",
            10
        )
        self.filter = ekf.ExtendedKalmanFilter()
        self.filter.setBeaconPosition(np.array([3, -2]))
        self.filter.setInitialPose(np.array([0, 0, 0]))
        self.filter.setInitialCovariance(np.zeros((3, 3)))
        self.lastCtrlTime = self.get_clock().now()

        self.yawReading = 0
        self.pointReading = np.array([3, -2])

        self.publish_timer = self.create_timer(0.1, self.publish_estimate)

    # msg : sensor_msgs.msg.Imu
    def imu_callback(self, msg):
        quat = [
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ]
        _, _, yaw = quat2euler(quat)
        self.yawReading = yaw
        # self.filter.update(np.array([self.pointReading[0], self.pointReading[1], self.yawReading]), np.eye(3))

    # msg : sensor_msgs.msg.PointCloud2
    def point_callback(self, msg):
        points = point_cloud2.read_points_list(msg)
        self.pointReading = points[0]
        # self.filter.update(np.array([self.pointReading[0], self.pointReading[1], self.yawReading]), np.eye(3))

    # now, msg : Float64
    # later, msg : geometry_msgs.msg.Twist
    def control_callback(self, msg):
        ctrlArray = np.array([msg.data[0], msg.data[1]])
        newTime = self.get_clock().now()
        timeDif = (newTime - self.lastCtrlTime).nanoseconds / 1000000000
        # self.filter.predict(ctrlArray, np.eye(3), timeDif)
        self.lastCtrlTime = newTime

    def publish_estimate(self):
        est = PoseWithCovarianceStamped()
        est.header.frame_id = "map"
        est.header.stamp = self.get_clock().now().to_msg()
        # est.pose = self.filter.toPoseWithCovariance()
        self.est_pub.publish(est)

def main(args=None):
    rclpy.init(args=args)
    node = EkfNodeWrapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()