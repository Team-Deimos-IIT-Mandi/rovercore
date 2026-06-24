import rclpy
from geometry_msgs.msg import Pose
from nav2_msgs.action import NavigateToPose

import yasmin
from yasmin import CbState, Blackboard, StateMachine
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL


class Nav2State(ActionState):
    def __init__(self) -> None:
        super().__init__(
            NavigateToPose,
            "/navigate_to_pose",
            self.create_goal_handler,
            [SUCCEED, ABORT, CANCEL],
            None,
        )

    def create_goal_handler(self, blackboard: Blackboard) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose.pose = blackboard["pose"]
        goal.pose.header.frame_id = "map"
        return goal


def create_nav_sm(x: float, y: float) -> StateMachine:
    nav_sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])

    def _set_pose(blackboard: Blackboard) -> str:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.w = 1.0
        blackboard["pose"] = pose
        yasmin.YASMIN_LOG_INFO(f"Set navigation target to ({x}, {y})")
        return SUCCEED

    nav_sm.add_state(
        "SET_POSE",
        CbState([SUCCEED], _set_pose),
        transitions={SUCCEED: "NAVIGATING"},
    )
    nav_sm.add_state(
        "NAVIGATING",
        Nav2State(),
        transitions={SUCCEED: SUCCEED, ABORT: ABORT, CANCEL: CANCEL},
    )

    return nav_sm
