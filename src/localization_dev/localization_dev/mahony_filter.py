import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import numpy as np


def quat_mul(q, r):
    """Quaternion multiplication q x r"""
    w1, x1, y1, z1 = q
    w2, x2, y2, z2 = r
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])


def quat_rotate(q, v):
    """Rotate vector v by quaternion q."""
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    v_quat = np.array([0.0, v[0], v[1], v[2]])
    return quat_mul(quat_mul(q, v_quat), q_conj)[1:]


class MahonyFilter(Node):
    def __init__(self):
        """
        Tunable params:
            kP : Proportional gain
            kI : Integral gain
        
        Need to specify intial attitude estimates, biases and sampling time

        Let Initial attitude be zero for at rest,
        Let biases be computed by taking average samples of IMU at rest
            then by taking the mean value
        The sampling time is the inverse of the IMU publishing rate

        NOTE: Bias drifts over time

        ros2 launch phidgets_spatial spatial-launch.py --> IMU publisher
        """
        super().__init__('mahony_filter')

        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # Initial quaternion
        self.bg = np.array([0.0, 0.0, 0.0])  # Inital gyro bias
        self.kP = 1.0
        self.kI = 0.01
        self.integral_error = np.zeros(3)

        self.last_time = None

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            50
        )
        
        # Needs to be implemented later
        # self.heading_sub = self.create_subscription(

        self.pub = self.create_publisher(
            Quaternion,
            '/imu/quaternion',
            50
        )

    def imu_callback(self, msg):
        """
        Gryo Model:
            w = w_hat _+ b_g + n_g
        w   : measured angular velocity
        w_hat: true angular velocity
        bg(t) : gyro bias -> bg_dot = bg(t) ~ N(0, Qg)
                                              Qg : covariance mtx gyro bias noise
        
        Accel Model:
            a = R.T(a_hat - g) + b_a + n_a
        
        a    : measured acceleration
        a_hat: true acceleration
        g    : gravity vector
        R    : orientation of the sensor wrt the world frame
        b_a(t): accel bias -> b_a_dot = b_a(t) ~ N(0, Qa)
                                              Qa : covariance mtx accel bias noise
        
        Orientation of the system must be known from an outside source 

        https://nitinjsanket.github.io/tutorials/attitudeest/mahony.html

        ---------------------------------------------------------------------------
        
        Defining a coord axis:

        Let I, W, B be the inertial, world, and body frames respectively.

        The goal is to estimate (I->W)q -> q is a quaternion
        
        https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/asl-dam/documents/lectures/robot_dynamics/RD2_Quaternions.pdf


        1) Obtain sensor measurements
            I^omega_t, I^a_t be gyro, accel measurements resp. and I^ahat_t be normed

        2) Orientation error using accel measurments

        Compute orientation error from previous estimate using accel measurements

        v (W->I qhat_est,t) = [2*(q2q4 - q1q3),
                               2*(q1q2 + q3q4),
                               (q1^2 - q2^2 - q3^2 + q4^2)]

        e_t+1 = I^ahat_t+1 x v(W->I qhat_est,t)   # for P
        ei_t+1 = ei_t + e_t+1 * dt                # for I

        dt is the time elapsed between samples

        3) Update gyro using PI compensations (Fusion)

        I^omega_t+1 = I^omega_t+1 + kP*e_t+1 + kI*ei_t+1

        4) Orientation increment from Gyro

        (I->W qdot_omega,t+1) = 1/2 * (W->I qhat_est,t) quat_mult [0, I^omega_t+1].T

        5) Numerical Integration

        Compute orientation using numerical integration

        (I->W q_est,t+1) = (I->W qhat_est,t) + (I->W qdot_omega,t+1) * dt

        For each time step, repeat from step 1
        ---------------------------------------------------------------------------

        """

        # --- Compute dt ---
        if self.last_time is None:
            self.last_time = msg.header.stamp
            return

        t = msg.header.stamp
        t_now = t.sec + t.nanosec*1e-9
        t_prev = self.last_time.sec + self.last_time.nanosec*1e-9
        dt = t_now - t_prev
        self.last_time = t
        if dt <= 0:
            return
        if dt > 0.5:
            return  # Ignore large time gaps

        # --- Read IMU ---
        # rad/s
        gyro = np.array([msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z]) - self.bg
        
        # m/s^2
        accel = np.array([msg.linear_acceleration.x,
                          msg.linear_acceleration.y,
                          msg.linear_acceleration.z])

        if np.linalg.norm(accel) == 0:
            return

        accel = accel / np.linalg.norm(accel)  # Only ahat is used

        # --- Compute orientation error ---
        v = quat_rotate(self.q, np.array([0.0, 0.0, -1.0]))  # Gravity vector in body frame
        
        e = np.cross(v, accel)
        self.integral_error += e * dt

        # --- Update gyro measurements ---
        gyro_corrected = gyro + self.kP * e + self.kI * self.integral_error

        # --- Orientation increment from gyro ---
        qdot_omega = 0.5 * quat_mul(
            self.q,
            np.array([0.0, gyro_corrected[0], gyro_corrected[1], gyro_corrected[2]]) # .T 1-d Array
        )

        # --- Numerical integration ---
        self.q = self.q + qdot_omega * dt  # q_t+1
        self.q = self.q / np.linalg.norm(self.q)  # Normalize
                                                  # Non-unit quaternions lead to errors
        # --- Publish quaternion ---
        quat_msg = Quaternion()
        quat_msg.w = self.q[0]
        quat_msg.x = self.q[1]
        quat_msg.y = self.q[2]
        quat_msg.z = self.q[3]
        self.pub.publish(quat_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MahonyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()