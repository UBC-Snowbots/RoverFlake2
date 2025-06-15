import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

import numpy as np

def euler_to_rot(roll, pitch, yaw):
    """
    Create rotation matrix from roll, pitch, yaw (ZYX convention).
    We assume +Z is up in the nav frame.
    """
    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [ 0,   0, 1]])
    Ry = np.array([[cp,  0, sp],
                   [ 0,  1,  0],
                   [-sp, 0, cp]])
    Rx = np.array([[1,  0,   0 ],
                   [0, cr,  -sr],
                   [0, sr,   cr]])
    return Rz @ Ry @ Rx

class GpsImuFusionNode(Node):
    def __init__(self):
        super().__init__('gps_imu_fusion_node')

        # State dimension: [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.n_x = 9

        # Initialize state mean and covariance
        self.x = np.zeros(self.n_x)     # [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.P = np.eye(self.n_x) * 1e-3  # small initial uncertainty

        # Process noise and measurement noise covariances (tune them!)
        self.Q = np.eye(self.n_x) * 1e-2  
        self.R = np.eye(3) * 2.0  # GPS measurement noise for x,y,z

        # UKF parameters
        self.alpha = 1e-3
        self.kappa = 0
        self.beta  = 2.0
        self.lambda_ = self.alpha**2 * (self.n_x + self.kappa) - self.n_x
        self.gamma = np.sqrt(self.n_x + self.lambda_)

        # Timing
        self.last_time_imu = None

        # Subscribers
        self.sub_imu = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.sub_gps = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )

        # Publishers
        self.pub_odom = self.create_publisher(Odometry, '/odom/fused', 10)
        self.pub_path = self.create_publisher(Path, '/odom/fused_path', 10)

        # Create a Path object to accumulate poses
        self.path = Path()
        self.path.header.frame_id = 'odom'  # or another frame you'd like

        self.gps_first = True
        self.gps_start_loc = None

        self.get_logger().info("GPS-IMU fusion node started.")

    # --------------------------------------------------------------------------
    # IMU callback: triggers the UKF prediction step
    # --------------------------------------------------------------------------
    def imu_callback(self, msg: Imu):
        if self.last_time_imu is None:
            self.last_time_imu = msg.header.stamp
            return

        current_time = msg.header.stamp
        dt = (current_time.sec - self.last_time_imu.sec) \
             + (current_time.nanosec - self.last_time_imu.nanosec)*1e-9
        self.last_time_imu = current_time

        # Extract raw accelerations, angular rates
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        self.ukf_predict(dt, ax, ay, az, gx, gy, gz)

    # --------------------------------------------------------------------------
    # GPS callback: triggers the measurement update
    # --------------------------------------------------------------------------
    def gps_callback(self, msg: NavSatFix):
        if np.isnan(msg.latitude) or np.isnan(msg.longitude):
            return  # ignore invalid GPS data

        # Convert lat/lon/alt to local (x, y, z)
        x_gps, y_gps, z_gps = self.latlonalt_to_enu(
            msg.latitude, msg.longitude, msg.altitude
        )

        if self.gps_first:
            self.gps_start_loc = np.array([x_gps, y_gps, z_gps])
            self.gps_first = False
        else :
            z_meas = np.array([x_gps, y_gps, z_gps]) - self.gps_start_loc
            self.ukf_update(z_meas)

            # Publish fused odometry + path
            self.publish_fused_odom()

    # --------------------------------------------------------------------------
    # 1) UKF Predict Step
    # --------------------------------------------------------------------------
    def ukf_predict(self, dt, ax, ay, az, gx, gy, gz):
        """
        1) Generate sigma points from (x, P)
        2) Propagate each through motion model
        3) Compute new mean, covariance
        """
        # Generate sigma points
        Xsigma = self.generate_sigma_points(self.x, self.P)

        # Propagate each sigma point
        Xsigma_pred = []
        for i in range(2*self.n_x + 1):
            Xsigma_pred.append(
                self.process_model_f(Xsigma[i], dt, ax, ay, az, gx, gy, gz)
            )
        Xsigma_pred = np.array(Xsigma_pred)  # shape: (2n+1, n_x)

        # Recompute mean x, cov P
        w_m, w_c = self.compute_weights()

        # Predicted state mean
        x_pred = np.zeros(self.n_x)
        for i in range(2*self.n_x + 1):
            x_pred += w_m[i] * Xsigma_pred[i]

        # Predicted covariance
        P_pred = np.zeros((self.n_x, self.n_x))
        for i in range(2*self.n_x + 1):
            dx = (Xsigma_pred[i] - x_pred).reshape(-1,1)
            P_pred += w_c[i] * (dx @ dx.T)
        P_pred += self.Q  # add process noise

        self.x = x_pred
        self.P = P_pred

    def process_model_f(self, state, dt, ax, ay, az, gx, gy, gz):
        """
        naive approach:
          - rotate (ax,ay,az)->nav frame => subtract gravity => integrate
          - integrate roll/pitch/yaw with gyro
        """
        # Unpack
        x, y, z, vx, vy, vz, roll, pitch, yaw = state

        # Rotate acceleration
        R_b2n = euler_to_rot(roll, pitch, yaw)
        a_body = np.array([ax, ay, az])
        a_nav  = R_b2n @ a_body
        a_nav[2] -= 9.81  # gravity

        # Integrate velocity & position
        vx_new = vx + a_nav[0]*dt
        vy_new = vy + a_nav[1]*dt
        vz_new = vz + a_nav[2]*dt

        x_new  = x  + vx_new*dt
        y_new  = y  + vy_new*dt
        z_new  = z  + vz_new*dt

        # Integrate orientation
        roll_new  = roll  + gx*dt
        pitch_new = pitch + gy*dt
        yaw_new   = yaw   + gz*dt
        # wrap yaw to [-pi, pi]
        yaw_new = (yaw_new + np.pi) % (2.0*np.pi) - np.pi

        return np.array([
            x_new, y_new, z_new,
            vx_new, vy_new, vz_new,
            roll_new, pitch_new, yaw_new
        ])

    # --------------------------------------------------------------------------
    # 2) UKF Update Step (GPS)
    # --------------------------------------------------------------------------
    def ukf_update(self, z_meas):
        """
        1) Generate sigma points from predicted (x, P)
        2) Transform them via measurement model h(X_i)
        3) Compute Kalman gain K
        4) Update x, P
        """
        Xsigma = self.generate_sigma_points(self.x, self.P)

        # Measurement predictions for each sigma point
        Zsigma = []
        for i in range(2*self.n_x + 1):
            Zsigma.append(self.measurement_model_h(Xsigma[i]))
        Zsigma = np.array(Zsigma)  # shape (2n+1, 3)

        # Weights
        w_m, w_c = self.compute_weights()

        # Predicted measurement mean
        z_pred = np.zeros(3)
        for i in range(2*self.n_x + 1):
            z_pred += w_m[i] * Zsigma[i]

        # Measurement covariance S and cross-covariance Pxz
        S = np.zeros((3,3))
        Pxz = np.zeros((self.n_x, 3))

        for i in range(2*self.n_x + 1):
            dz = (Zsigma[i] - z_pred).reshape(-1,1)
            dx = (Xsigma[i] - self.x).reshape(-1,1)
            S += w_c[i] * (dz @ dz.T)
            Pxz += w_c[i] * (dx @ dz.T)

        S += self.R  # add measurement noise

        # Kalman gain
        K = Pxz @ np.linalg.inv(S)

        # Innovation
        innovation = (z_meas - z_pred).reshape(-1,1)

        # Update state and covariance
        self.x = self.x + (K @ innovation).ravel()
        self.P = self.P - K @ S @ K.T

    def measurement_model_h(self, state):
        """
        For GPS, h(x) = x[:3]
        """
        return state[:3]

    # --------------------------------------------------------------------------
    # Sigma Points and Weights
    # --------------------------------------------------------------------------
    def generate_sigma_points(self, x, P):
        A = np.linalg.cholesky((self.n_x + self.lambda_)*P)
        Xsigma = []
        Xsigma.append(x)

        for i in range(self.n_x):
            Xsigma.append(x + A[:,i])
        for i in range(self.n_x):
            Xsigma.append(x - A[:,i])
        return Xsigma

    def compute_weights(self):
        n_sigma = 2*self.n_x + 1
        w_m = np.zeros(n_sigma)
        w_c = np.zeros(n_sigma)

        w_m[0] = self.lambda_ / (self.n_x + self.lambda_)
        w_c[0] = self.lambda_ / (self.n_x + self.lambda_) + (1 - self.alpha**2 + self.beta)

        for i in range(1, n_sigma):
            w_m[i] = 1.0 / (2.0*(self.n_x + self.lambda_))
            w_c[i] = 1.0 / (2.0*(self.n_x + self.lambda_))

        return w_m, w_c

    # --------------------------------------------------------------------------
    # Utility: lat/lon/alt to local
    # --------------------------------------------------------------------------
    def latlonalt_to_enu(self, lat, lon, alt):
        # """
        # Dummy placeholder. In practice, you'd do:
        #   1) LLA -> ECEF
        #   2) subtract reference
        #   3) ECEF -> ENU (or NED)
        # """
        # # Example hack: offset from (lat=50, lon=8)
        # x_local = (lon - 8.0) * 10000.0
        # y_local = (lat - 50.0) * 10000.0
        # z_local = alt - 300.0
        # return x_local, y_local, z_local
        # Convert degrees to radians
        # Convert degrees to radians
        lat_rad = np.radians(lat)
        lon_rad = np.radians(lon)
        
        # WGS84 ellipsoid constants
        a = 6378137.0             # semi-major axis in meters
        f = 1 / 298.257223563     # flattening
        e2 = 2 * f - f**2         # eccentricity squared
        
        # Compute prime vertical radius of curvature
        N = a / np.sqrt(1 - e2 * np.sin(lat_rad)**2)
        
        # Convert LLA to ECEF coordinates
        x = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
        y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
        z = (N * (1 - e2) + alt) * np.sin(lat_rad)
        
        # Compute the rotation matrix for ENU conversion based on the input lat, lon.
        # Standard ENU rotation matrix is:
        #   [ -sin(lon),              cos(lon),               0;
        #     -sin(lat)*cos(lon), -sin(lat)*sin(lon), cos(lat);
        #      cos(lat)*cos(lon),  cos(lat)*sin(lon), sin(lat) ]
        R = np.array([
            [-np.sin(lon_rad),              np.cos(lon_rad),               0],
            [-np.sin(lat_rad)*np.cos(lon_rad), -np.sin(lat_rad)*np.sin(lon_rad), np.cos(lat_rad)],
            [ np.cos(lat_rad)*np.cos(lon_rad),  np.cos(lat_rad)*np.sin(lon_rad), np.sin(lat_rad)]
        ])
        
        # Form the ECEF vector
        ecef = np.array([x, y, z])
        
        # Apply the rotation matrix to obtain ENU coordinates
        enu = R @ ecef
        e, n, u = enu[0], enu[1], enu[2]
        
        return e, n, u

    # --------------------------------------------------------------------------
    # Publish the fused state as an Odometry message *and* update the path
    # --------------------------------------------------------------------------
    def publish_fused_odom(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        # Fill position
        odom_msg.pose.pose.position.x = self.x[0]
        odom_msg.pose.pose.position.y = self.x[1]
        odom_msg.pose.pose.position.z = self.x[2]

        # Convert roll/pitch/yaw => quaternion (ZYX)
        cr = np.cos(self.x[6]*0.5)
        sr = np.sin(self.x[6]*0.5)
        cp = np.cos(self.x[7]*0.5)
        sp = np.sin(self.x[7]*0.5)
        cy = np.cos(self.x[8]*0.5)
        sy = np.sin(self.x[8]*0.5)

        qw = cr*cp*cy + sr*sp*sy
        qx = sr*cp*cy - cr*sp*sy
        qy = cr*sp*cy + sr*cp*sy
        qz = cr*cp*sy - sr*sp*cy

        odom_msg.pose.pose.orientation.w = qw
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz

        # Fill velocity
        odom_msg.twist.twist.linear.x = self.x[3]
        odom_msg.twist.twist.linear.y = self.x[4]
        odom_msg.twist.twist.linear.z = self.x[5]

        self.pub_odom.publish(odom_msg)

        # ------------------ Update Path ------------------
        pose_stamped = PoseStamped()
        pose_stamped.header = odom_msg.header
        pose_stamped.pose = odom_msg.pose.pose

        self.get_logger().info(f"Publishing fused odometry: {self.x[:3]}")

        # Update path's header (the stamp)
        self.path.header.stamp = odom_msg.header.stamp
        self.path.poses.append(pose_stamped)

        # Publish path
        self.pub_path.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = GpsImuFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
