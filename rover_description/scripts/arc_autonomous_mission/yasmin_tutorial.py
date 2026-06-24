import random
import rclpy
from geometry_msgs.msg import Pose
from nav2_msgs.action import NavigateToPose

import yasmin
from yasmin import CbState, Blackboard, StateMachine
from yasmin_ros import ActionState
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

# Constants for state outcomes
HAS_NEXT = "has_next"  ##< Indicates there are more waypoints
END = "end"  ##< Indicates no more waypoints

SEQUENCE = [
    "BOOTING",
    "NAVIGATE_TO_SENSOR_ZONE_1", "DOME_RETURN", "DELIVER_RECORDER",
    "NAVIGATE_TO_SENSOR_ZONE_2", "DOME_RETURN","DELIVER_SENSORS",
    "NAVIGATE_TO_PANEL_ZONE",
    "NAVIGATE_TO_PIPE_ZONE",
    # "NAVIGATE_TO_MIRROR_ZONE",
    # "NAVIGATE_TO_ANTENNA_ZONE",
    # "NAVIGATE_TO_LAVA_ZONE",
    "DOME_RETURN",
    "SEQUENCE_COMPLETE",
]

WAYPOINTS = {
    "NAVIGATE_TO_SENSOR_ZONE_1": (6.0, 10.0),
    "NAVIGATE_TO_SENSOR_ZONE_2": (6.0, 10.0),
    "NAVIGATE_TO_PANEL_ZONE": (16.0, 26.0),
    "NAVIGATE_TO_PIPE_ZONE": (10.0, 28.0),
    # "NAVIGATE_TO_MIRROR_ZONE": (15.0, -10.0),
    # "NAVIGATE_TO_ANTENNA_ZONE": (5.0, -10.0),
    # "NAVIGATE_TO_LAVA_ZONE": (0.0, -15.0),
    "DOME_RETURN": (10.0, 0.0),
}
class Nav2State(ActionState):
    """
    ActionState for navigating to a specified pose using ROS 2 Navigation.
    """

    def __init__(self) -> None:
        """
        Initializes the Nav2State.

        Calls the parent constructor to set up the action with:
        - Action type: NavigateToPose
        - Action name: /navigate_to_pose
        - Callback for goal creation: create_goal_handler
        - Outcomes: None, since it will use default outcomes (SUCCEED, ABORT, CANCEL)
        """
        super().__init__(
            NavigateToPose,  # action type
            "/navigate_to_pose",  # action name
            self.create_goal_handler,  # callback to create the goal
            [SUCCEED, ABORT, CANCEL],  # Pass the outcomes list instead of None
            None,
        )

    def create_goal_handler(self, blackboard: Blackboard) -> NavigateToPose.Goal:
        """
        Creates a goal for navigation based on the current pose in the blackboard.

        Args:
            blackboard (Blackboard): The blackboard instance holding current state data.

        Returns:
            NavigateToPose.Goal: The constructed goal for the navigation action.
        """
        goal = NavigateToPose.Goal()
        goal.pose.pose = blackboard["pose"]
        goal.pose.header.frame_id = "map"  # Set the reference frame to 'map'
        return goal


def create_waypoints(blackboard: Blackboard) -> str:
    """
    Initializes waypoints in the blackboard for navigation.

    Args:
        blackboard (Blackboard): The blackboard instance to store waypoints.

    Returns:
        str: Outcome indicating success (SUCCEED).
    """
    blackboard["waypoints"] = {
        "entrance": [1.25, 6.30, -0.78, 0.67],
        "bathroom": [4.89, 1.64, 0.0, 1.0],
        "livingroom": [1.55, 4.03, -0.69, 0.72],
        "kitchen": [3.79, 6.77, 0.99, 0.12],
        "bedroom": [7.50, 4.89, 0.76, 0.65],
    }
    return SUCCEED

def take_random_waypoint(blackboard: Blackboard) -> str:
    """
    Selects a random set of waypoints from the available waypoints.

    Args:
        blackboard (Blackboard): The blackboard instance to store random waypoints.

    Returns:
        str: Outcome indicating success (SUCCEED).
    """
    blackboard["random_waypoints"] = random.sample(
        list(blackboard["waypoints"].keys()), blackboard["waypoints_num"]
    )
    return SUCCEED

def get_next_waypoint(blackboard: Blackboard) -> str:
    """
    Retrieves the next waypoint from the list of random waypoints.

    Updates the blackboard with the pose of the next waypoint.

    Args:
        blackboard (Blackboard): The blackboard instance holding current state data.

    Returns:
        str: Outcome indicating whether there is a next waypoint (HAS_NEXT) or if
             navigation is complete (END).
    """
    if not blackboard["random_waypoints"]:
        return END

    wp_name = blackboard["random_waypoints"].pop(0)  # Get the next waypoint name
    wp = blackboard["waypoints"][wp_name]  # Get the waypoint coordinates

    pose = Pose()
    pose.position.x = wp[0]
    pose.position.y = wp[1]
    pose.orientation.z = wp[2]
    pose.orientation.w = wp[3]

    blackboard["pose"] = pose  # Update blackboard with new pose
    blackboard["text"] = f"I have reached waypoint {wp_name}"

    return HAS_NEXT


def main() -> None:
    # Initialize ROS 2
    rclpy.init()

    # Set ROS 2 loggers
    set_ros_loggers()
    yasmin.YASMIN_LOG_INFO("yasmin_nav2_demo")

    # Create state machines
    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL], handle_sigint=True)
    nav_sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])

    # Add states to the state machine
    sm.add_state(
        "CREATING_WAYPOINTS",
        CbState([SUCCEED], create_waypoints),
        transitions={
            SUCCEED: "TAKING_RANDOM_WAYPOINTS",
        },
    )
    sm.add_state(
        "TAKING_RANDOM_WAYPOINTS",
        CbState([SUCCEED], take_random_waypoint),
        transitions={
            SUCCEED: "NAVIGATING",
        },
    )
    nav_sm.add_state(
        "GETTING_NEXT_WAYPOINT",
        CbState([END, HAS_NEXT], get_next_waypoint),
        transitions={
            END: SUCCEED,
            HAS_NEXT: "NAVIGATING",
        },
    )
    nav_sm.add_state(
        "NAVIGATING",
        Nav2State(),
        transitions={
            SUCCEED: "GETTING_NEXT_WAYPOINT",
            CANCEL: CANCEL,
            ABORT: ABORT,
        },
    )
    sm.add_state(
        "NAVIGATING",
        nav_sm,
        transitions={
            SUCCEED: SUCCEED,
            CANCEL: CANCEL,
            ABORT: ABORT,
        },
    )

    # Publish FSM information for visualization
    YasminViewerPub(sm, "YASMIN_NAV2_DEMO")

    # Set the number of waypoints
    blackboard = Blackboard()
    blackboard["waypoints_num"] = 2

    # Execute the FSM
    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()