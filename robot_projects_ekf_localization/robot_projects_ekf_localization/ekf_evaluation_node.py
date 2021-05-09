import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from transforms3d.euler import quat2euler
import numpy as np
import sys

def poseToState(pose):
    state = np.zeros(3)
    state[0] = pose.position.x
    state[1] = pose.position.y

    # convert pose quaternion to rpy for heading
    quat = [
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z
    ]

    _, _, state[2] = quat2euler(quat)
    return state

def poseCovToStateCov(poseCov):
    stateCov = np.zeros((3, 3))

    stateCov[0][0] = poseCov[0]
    stateCov[0][1] = poseCov[1]
    stateCov[0][2] = poseCov[5]
    stateCov[1][0] = poseCov[6]
    stateCov[1][1] = poseCov[7]
    stateCov[1][2] = poseCov[11]
    stateCov[2][0] = poseCov[30]
    stateCov[2][1] = poseCov[31]
    stateCov[2][2] = poseCov[35]

    return stateCov

# returns the value of the multivariate normal PDF with mean 0 and covariance cov at x
def probability(cov, x):
    return np.exp(x.T @ np.linalg.inv(cov) @ x * -0.5) / np.sqrt(np.power(2 * np.pi, len(x)) * np.linalg.det(cov))

# the natural log of the multivariate normal PDF defined above, for numerical stability
def logProbability(cov, x):
    return (x.T @ np.linalg.inv(cov) @ x * -0.5) - np.log(np.sqrt(np.power(2 * np.pi, len(x)) * np.linalg.det(cov)))

class EkfEvaluationNode(Node):
    def __init__(self):
        super().__init__("ekf_evaluation")
        self.declare_parameter("eval_output")

        self.est_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "pose_estimate",
            self.est_callback,
            10
        )
        self.truth_sub = self.create_subscription(
            PoseStamped,
            "pose_sensor/reading",
            self.truth_callback,
            10
        )

        self.time_start = None
        self.last_truth = None
        self.last_est = None
        self.probabilities = []
        outfile_name = self.get_parameter("eval_output").get_parameter_value().string_value
        if outfile_name != "":
            self.get_logger().info("Writing output to " + outfile_name)
            self.outfile = open(outfile_name, "wt")
        else:
            self.get_logger().info("No output file given, writing to screen")
            self.outfile = None

    # msg :: PoseWithCovarianceStamped
    def est_callback(self, msg):
        self.last_est = msg

    # msg :: PoseStamped
    def truth_callback(self, msg):
        if self.time_start == None:
            self.time_start = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
            self.get_logger().info("Found starting time (ns): " + str(self.time_start))

        if self.last_truth != None and self.last_est != None:
            time_last = rclpy.time.Time.from_msg(self.last_truth.header.stamp)
            time_now = rclpy.time.Time.from_msg(msg.header.stamp)
            time_est = rclpy.time.Time.from_msg(self.last_est.header.stamp)
            t = (time_est - time_last).nanoseconds / (time_now - time_last).nanoseconds

            truth_interpolated = t * poseToState(msg.pose) + (1 - t) * poseToState(self.last_truth.pose)
            error = truth_interpolated - poseToState(self.last_est.pose.pose)
            cov = poseCovToStateCov(self.last_est.pose.covariance)

            time_est_float = (time_now.nanoseconds - self.time_start) / 1000000000
            p = probability(cov, error)
            lnp = logProbability(cov, error)
            if self.outfile != None:
                self.outfile.write(str(time_est_float) + ", " + str(p) + ", " + str(lnp) + "\n")
            else:
                self.get_logger().info(str(p))

        self.last_truth = msg

def main(args=None):
    rclpy.init(args=args)
    node = EkfEvaluationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
