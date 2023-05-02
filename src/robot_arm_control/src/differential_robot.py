import math
class DifferentialRobot:
    def __init__(self, wheel_radius, wheel_distance):
        self.wheel_radius = wheel_radius  # radius of wheels in meters
        self.wheel_distance = wheel_distance  # distance between wheels in meters
        self.left_wheel_velocity = 0.0  # angular velocity of left wheel in radians per second
        self.right_wheel_velocity = 0.0  # angular velocity of right wheel in radians per second
        self.x = 0.0  # x position of the robot in meters
        self.y = 0.0  # y position of the robot in meters
        self.theta = 0.0  # orientation of the robot in radians

    def update_pose(self, delta_t):
        # Calculate linear and angular velocities of the robot
        linear_velocity = self.wheel_radius / 2.0 * (self.left_wheel_velocity + self.right_wheel_velocity)
        angular_velocity = self.wheel_radius / self.wheel_distance * (self.right_wheel_velocity - self.left_wheel_velocity)

        # Calculate change in pose
        delta_x = linear_velocity * math.cos(self.theta) * delta_t
        delta_y = linear_velocity * math.sin(self.theta) * delta_t
        delta_theta = angular_velocity * delta_t

        # Update pose
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

    def set_wheel_velocities(self, left_wheel_velocity, right_wheel_velocity):
        self.left_wheel_velocity = left_wheel_velocity
        self.right_wheel_velocity = right_wheel_velocity

