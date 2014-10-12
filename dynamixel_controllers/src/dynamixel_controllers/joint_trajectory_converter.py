__author__ = 'Andrew Price'

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class JointConverter:
    def __init__(self, name, reverse, offset):
        self.joint_name = name
        if reverse:
            self.multiplier = -1.0
        else:
            self.multiplier = 1.0
        self.offset = offset

    def convert_angle(self, urdf_angle):
        """ Converts URDF angle to joint angle
        Args:
            urdf_angle: joint angle in URDF file
        Returns:
            The joint angle
        """
        return (urdf_angle * self.multiplier) + self.offset

    def convert_velocity(self, urdf_velocity):
        """ Converts URDF velocity to joint velocity
        Args:
            urdf_velocity
        Returns:
            The velocity
        """
        return urdf_velocity * self.multiplier


class TrajectoryConverter:
    def __init__(self):
        self.joint_map = {}

    # Default (straight-through) joint
    def add_joint(self, urdf_name):
        self.joint_map[urdf_name] = JointConverter(urdf_name, False, 0)

    def add_joint(self, urdf_name, dynamixel_name, reverse, offset):
        """ Adds a Joint Converter to the joint mapping
        Args:
            urdf_name: URDF file name
            dynamixel_name: dynamixel motor type
            reverse:
            offset:
        """
        self.joint_map[urdf_name] = JointConverter(dynamixel_name, reverse, offset)

    def convert_trajectory(self, traj):
        """ Converts a trajectory into a joint trajectory
        Args:
            traj: Trajectory to convert
        Returns:
            joint trajectory
        """
        new_traj = JointTrajectory()

        new_traj.header = traj.header
        # Take each joint in the trajectory and add it to the joint trajectory
        for joint_name in traj.joint_names:
            new_traj.joint_names.append(self.joint_map[joint_name].joint_name)
        # For each poiint in the trajectory
        for point in traj.points:
            new_point = JointTrajectoryPoint()
            for i, pos in enumerate(point.positions):
                new_point.positions.append(self.joint_map[new_traj.joint_names[i]].convert_angle(pos))
            for i, vel in enumerate(point.velocities):
                new_point.velocities.append(self.joint_map[new_traj.joint_names[i]].convert_velocity(vel))

            new_point.time_from_start = point.time_from_start
            new_traj.points.append(new_point)

        return new_traj