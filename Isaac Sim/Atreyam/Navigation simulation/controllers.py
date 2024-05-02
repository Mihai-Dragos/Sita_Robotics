from omni.isaac.core.controllers import BaseController
from omni.isaac.core.utils.types import ArticulationAction

# Used for velocity control
class CoolController(BaseController):
    def __init__(self):
        super().__init__(name="my_cool_controller")
       # An open loop controller that uses a unicycle model
        self._wheel_radius = 0.03
        self._wheel_base = 0.1125
        return

    def forward(self, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        joint_velocities = [0.0, 0.0]
        joint_velocities[0] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities=joint_velocities)

# Trying to convert the input: vector velocity to output steps doable by robot
class MyHoloController(BaseController):
    def __init__(self):
        super().__init__(name="my_holo_controller")
        self._wheel_radius = 0.03
        self._wheel_base = 0.1125
        return

    def vector_vel(self, vel):
        # command will have 3 elements, velocity in [x,y,z].
        # second element is the angular velocity (yaw only).
        
        command = [0.0, 0.0]
        command[0] = ((vel[0] ** 2) + (vel[1] ** 2)) ** 0.5                     # robot velocity amplitude
        # command[1] = get_robot_ori_euler(robot_index) - np.arctan2(vel[1],vel[0])   # robot direction (rad)
        joint_velocities = [0.0, 0.0]
        joint_velocities[0] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities=joint_velocities)
