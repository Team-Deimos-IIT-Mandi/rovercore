import os
import sys

import rclpy
import yasmin
from yasmin import CbState, Blackboard, StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mission_sequence import SEQUENCE, WAYPOINTS
from nav_state import create_nav_sm
from task_callbacks import TASK_CALLBACKS


def main() -> None:
    rclpy.init()
    set_ros_loggers()
    yasmin.YASMIN_LOG_INFO("ARC Autonomous Mission")

    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL], handle_sigint=True)

    for i, step in enumerate(SEQUENCE):
        is_last = i == len(SEQUENCE) - 1
        next_target = SUCCEED if is_last else SEQUENCE[i + 1]

        if step in WAYPOINTS:
            x, y = WAYPOINTS[step]
            sm.add_state(
                step,
                create_nav_sm(x, y),
                transitions={
                    SUCCEED: next_target,
                    ABORT: ABORT,
                    CANCEL: CANCEL,
                },
            )
        else:
            sm.add_state(
                step,
                CbState([SUCCEED], TASK_CALLBACKS[step]),
                transitions={SUCCEED: next_target},
            )

    YasminViewerPub(sm, "ARC_AUTONOMOUS_MISSION")

    blackboard = Blackboard()
    blackboard["waypoints"] = WAYPOINTS

    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(f"Mission outcome: {outcome}")
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(f"Mission exception: {e}")

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
