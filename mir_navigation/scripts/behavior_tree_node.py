#!/usr/bin/env python3

import rospy
import py_trees
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool
from py_trees.common import Status, ParallelPolicy

class MoveURRobot(py_trees.behaviour.Behaviour):
    def __init__(self, name, target_pose):
        super(MoveURRobot, self).__init__(name)
        self.target_pose = target_pose
        self.move_pub = None
        self.move_completed = False

    def setup(self, **kwargs):
        self.move_pub = rospy.Publisher('/ur_move', Pose, queue_size=10)
        self.move_completed_sub = rospy.Subscriber('/ur_move_completed', Bool, self.move_completed_callback)
        return True

    def initialise(self):
        self.move_completed = False

    def update(self):
        if not self.move_completed:
            pose_msg = Pose()
            pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = self.target_pose[:3]
            pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z = self.target_pose[3:]
            self.move_pub.publish(pose_msg)
            return Status.RUNNING
        return Status.SUCCESS

    def move_completed_callback(self, msg):
        self.move_completed = msg.data

class MoveRobotiqGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name, action):
        super(MoveRobotiqGripper, self).__init__(name)
        self.action = action
        self.action_pub = None
        self.action_completed = False

    def setup(self, **kwargs):
        self.action_pub = rospy.Publisher(f'/gripper_{self.action}', Bool, queue_size=10)
        self.action_completed_sub = rospy.Subscriber('/gripper_action_completed', Bool, self.action_completed_callback)
        return True

    def initialise(self):
        self.action_completed = False

    def update(self):
        if not self.action_completed:
            self.action_pub.publish(Bool(True))
            return Status.RUNNING
        return Status.SUCCESS

    def action_completed_callback(self, msg):
        self.action_completed = msg.data

class MoveMiRPlatform(py_trees.behaviour.Behaviour):
    def __init__(self, name, target_pose):
        super(MoveMiRPlatform, self).__init__(name)
        self.target_pose = target_pose
        self.move_pub = None
        self.move_completed = False

    def setup(self, **kwargs):
        self.move_pub = rospy.Publisher('/mir_move', PoseStamped, queue_size=10)
        self.move_completed_sub = rospy.Subscriber('/mir_move_completed', Bool, self.move_completed_callback)
        return True

    def initialise(self):
        self.move_completed = False

    def update(self):
        if not self.move_completed:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = self.target_pose[:3]
            pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = self.target_pose[3:]
            self.move_pub.publish(pose_msg)
            return Status.RUNNING
        return Status.SUCCESS

    def move_completed_callback(self, msg):
        self.move_completed = msg.data

def create_behavior_tree():
    # Define target positions
    ur1_targets = {
        "first": [0.3131517670483043, -0.48248385920024783, 0.25897901411322294, 1.2120123376156002, -2.8787753861318572, 0.05135493588867255],
        "second": [0.3240825960112341, -0.4397587344426857, 0.7457010706788806, -1.1419003557941345, 2.838165794910607, 0.07054400748933255],
        "third": [-0.22450336057595344, -0.6401904169012413, 0.6775683634747155, -0.6405269066982309, -2.908397283938638, 0.11801436089304941],
        "fourth": [-0.7709322761090227, -0.1573739929082766, 0.580240777107713, -2.06656379758062, -2.20878100236245, 0.0977290206490596],
        "fifth": [-0.8253358280917288, 0.0070666727467619235, -0.07522779649184129, -2.3150052799834064, -1.9875063865912799, 0.1732557687690799],
        "sixth": [-0.3963962432442024, -0.3412502318481732, 0.45201772824084124, 1.2963043483067056, 2.7727029582834186, 0.054898281518343715]
    }

    mir_targets = {
        "pos1": [5.322, 27.188, 0.0, 0.0, 0.0, -0.644, 0.765],
        "pos2": [6.170, 28.230, 0.0, 0.0, 0.0, 0.078, 0.997],
        "pos3": [7.548, 28.258, 0.0, 0.0, 0.0, 0.070, 0.998],
        "pos4": [8.908, 27.654, 0.0, 0.0, 0.0, -0.669, 0.743],
        "move_away": [5.322, 27.188, 0.0, 0.0, 0.0, -0.644, 0.765]
    }

    # Create behavior tree
    root = py_trees.composites.Sequence("Main Sequence", memory=True)

    # Initial parallel movement
    parallel_initial = py_trees.composites.Parallel("Initial Parallel Movement", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    
    ur_initial_sequence = py_trees.composites.Sequence("UR Initial Sequence", memory=True)
    ur_initial_sequence.add_children([
        MoveURRobot("UR to First", ur1_targets["first"]),
        MoveRobotiqGripper("Close Gripper", "close"),
        MoveURRobot("UR to Second", ur1_targets["second"]),
        MoveURRobot("UR to Third", ur1_targets["third"]),
        MoveURRobot("UR to Fourth", ur1_targets["fourth"])
    ])

    mir_initial_sequence = py_trees.composites.Sequence("MiR Initial Sequence", memory=True)
    mir_initial_sequence.add_children([
        MoveMiRPlatform("MiR to Pos1", mir_targets["pos1"]),
        MoveMiRPlatform("MiR to Pos2", mir_targets["pos2"]),
        MoveMiRPlatform("MiR to Pos3", mir_targets["pos3"]),
        MoveMiRPlatform("MiR to Pos4", mir_targets["pos4"])
    ])

    parallel_initial.add_children([ur_initial_sequence, mir_initial_sequence])

    # Intermediate sequence
    intermediate_sequence = py_trees.composites.Sequence("Intermediate Sequence", memory=True)
    intermediate_sequence.add_children([
        MoveURRobot("UR to Fifth (1)", ur1_targets["fifth"]),
        MoveRobotiqGripper("Open Gripper", "open"),
        MoveURRobot("UR to Sixth (1)", ur1_targets["sixth"]),
        MoveMiRPlatform("MiR Move Away (1)", mir_targets["move_away"])
    ])

    # MiR return sequence
    mir_return_sequence = py_trees.composites.Sequence("MiR Return Sequence", memory=True)
    mir_return_sequence.add_children([
        MoveMiRPlatform("MiR Return to Pos1", mir_targets["pos1"]),
        MoveMiRPlatform("MiR Return to Pos2", mir_targets["pos2"]),
        MoveMiRPlatform("MiR Return to Pos3", mir_targets["pos3"]),
        MoveMiRPlatform("MiR Return to Pos4", mir_targets["pos4"])
    ])

    # Final UR sequence
    final_ur_sequence = py_trees.composites.Sequence("Final UR Sequence", memory=True)
    final_ur_sequence.add_children([
        MoveURRobot("UR to Fifth (2)", ur1_targets["fifth"]),
        MoveRobotiqGripper("Close Gripper (2)", "close"),
        MoveURRobot("UR to Sixth (2)", ur1_targets["sixth"])
    ])

    # Add all sequences to the root
    root.add_children([
        parallel_initial,
        intermediate_sequence,
        mir_return_sequence,
        final_ur_sequence,
        MoveMiRPlatform("Final MiR Move Away", mir_targets["move_away"])
    ])

    return root

class BehaviorTreeNode:
    def __init__(self):
        rospy.init_node('behavior_tree_node')
        self.tree = py_trees.trees.BehaviourTree(create_behavior_tree())
        self.tree.setup(timeout=15)

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            self.tree.tick()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = BehaviorTreeNode()
        node.run()
    except rospy.ROSInterruptException:
        pass