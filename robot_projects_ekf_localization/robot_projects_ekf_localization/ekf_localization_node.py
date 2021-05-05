import rclpy
from sensor_msgs.msg import Imu, PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from transforms3d.euler import quat2euler
import pcl_msgs
from robot_projects_ekf_localization import point_cloud2, ekf
import numpy as np

assumedActCov = np.eye(3)
assumedSenseCov = np.eye(3) * 0.05

class EkfNodeWrapper(Node):
    def __init__(self):
        super().__init__("ekf_localization")
        self.imu_sub = self.create_subscription(
            Imu,
            "/imu/reading",
            self.imu_callback,
            10
        )
        self.point_sub = self.create_subscription(
            PointCloud2,
            "/pt_sensor/reading",
            self.point_callback,
            10
        )
        self.ctrl_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.control_callback,
            10
        )
        self.est_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "pose_estimate",
            10
        )
        self.filter = ekf.ExtendedKalmanFilter()
        self.filter.setBeaconPosition(np.array([3, -2]))
        self.filter.setInitialPose(np.array([0, 0, 0], dtype="float64"))
        self.filter.setInitialCovariance(np.zeros((3, 3), dtype="float64"))
        self.lastCtrlTime = self.get_clock().now()

        self.yawReading = 0
        self.pointReading = np.array([3, -2])

        self.publish_timer = self.create_timer(0.1, self.publish_estimate)

        self.last_ctrl = np.zeros(2, dtype="float64")
        self.update_dt = 0.01
        self.update_cov = assumedActCov * self.update_dt * self.update_dt
        self.update_timer = self.create_timer(self.update_dt, self.update_estimate)
        self.predict_cov = assumedSenseCov

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
        self.predict_cov[2][2] = msg.orientation_covariance[8]
        self.update()

    # msg : sensor_msgs.msg.PointCloud2
    def point_callback(self, msg):
        points = point_cloud2.read_points_list(msg)
        self.pointReading = points[0]
        self.update()

    def update(self):
        z = np.array([self.pointReading[0], self.pointReading[1], self.yawReading])
        self.get_logger().info("\n" + str(z) + "\n" + str(self.filter.h()) + "\n" + str(z - self.filter.h()))
        self.filter.update(z, self.predict_cov)

    # later, msg : geometry_msgs.msg.Twist
    def control_callback(self, msg):
        self.last_ctrl = np.array([msg.linear.x, msg.angular.z])

    def update_estimate(self):
        self.filter.predict(self.last_ctrl, self.update_cov, self.update_dt)

    def publish_estimate(self):
        est = PoseWithCovarianceStamped()
        est.header.frame_id = "map"
        est.header.stamp = self.get_clock().now().to_msg()
        est.pose = self.filter.toPoseWithCovariance()
        self.est_pub.publish(est)

def main(args=None):
    rclpy.init(args=args)
    node = EkfNodeWrapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()