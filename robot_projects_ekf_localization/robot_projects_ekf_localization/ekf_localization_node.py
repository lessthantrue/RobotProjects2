import rclpy
from sensor_msgs.msg import Imu, PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from transforms3d.euler import quat2euler
from robot_projects_ekf_localization import point_cloud2, ekf, ukf, particle_filter
import numpy as np

assumedActCov = np.eye(3)
assumedSenseCov = np.eye(3) * 0.05

PARAM_FILTER_TYPE = 'filter_type'

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
        
        self.declare_parameter(PARAM_FILTER_TYPE)

        filterType = self.get_parameter(PARAM_FILTER_TYPE).value
        if filterType == 'ekf': 
            self.filter = ekf.ExtendedKalmanFilter()
            self.get_logger().info("using extended kalman filter")
        elif filterType == 'ukf':
            self.filter = ukf.UnscentedKalmanFilter()
            self.get_logger().info("using unscented kalman filter")
        elif filterType == 'particle':
            self.filter = particle_filter.ParticleFilter()
            self.get_logger().info("using particle filter")
        else:
            self.get_logger().error("WARNING: NO FILTER SET ({} = {})".format(PARAM_FILTER_TYPE, filterType))
            
        if(self.filter.canVisualize()):
            self.vis_pub = self.create_publisher(
                    self.filter.getVisualizationType(),
                    "visualize_estimate",
                    10
                )
            self.visualize_timer = self.create_timer(0.1, self.publish_visualization)
            
        self.filter.setBeaconPosition(np.array([3, -2]))
        self.filter.setInitialPose(np.array([0, 0, 0], dtype="float64"))
        self.filter.setInitialCovariance(np.eye(3, dtype="float64") * 0.1)

        self.yawReading = 0
        self.pointReading = np.array([3, -2])

        self.publish_timer = self.create_timer(0.1, self.publish_estimate)

        self.last_ctrl = np.zeros(2, dtype="float64")
        self.update_dt = 0.01
        self.update_cov = assumedActCov * self.update_dt * self.update_dt * 1.5
        self.update_timer = self.create_timer(self.update_dt, self.update_estimate)
        self.sense_cov = assumedSenseCov

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
        self.sense_cov = np.copy(assumedSenseCov)
        self.sense_cov[2][2] = msg.orientation_covariance[8]
        self.sense_cov *= 1.5
        self.update(ignoreIndices=[0, 1])

    # msg : sensor_msgs.msg.PointCloud2
    def point_callback(self, msg):
        points = point_cloud2.read_points_list(msg)
        self.pointReading = points[0]
        self.update(ignoreIndices=[2])

    def update(self, ignoreIndices=[]):
        z = np.array([self.pointReading[0], self.pointReading[1], self.yawReading])
        self.filter.update(z, self.sense_cov, ignoreIndices=ignoreIndices)

    # msg : geometry_msgs.msg.Twist
    def control_callback(self, msg):
        minctrl = np.array([-1, -1])
        maxctrl = np.array([1, 1])
        self.last_ctrl = np.minimum(maxctrl, np.maximum(minctrl, np.array([msg.linear.x, msg.angular.z])))

    def update_estimate(self):
        self.filter.predict(self.last_ctrl, self.update_cov, self.update_dt)

    def publish_estimate(self):
        est = PoseWithCovarianceStamped()
        est.header.frame_id = "map"
        est.header.stamp = self.get_clock().now().to_msg()
        est.pose = self.filter.toPoseWithCovariance()
        self.est_pub.publish(est)
        
    def publish_visualization(self):
        msg = self.filter.getVisualizationData()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        self.vis_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EkfNodeWrapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()