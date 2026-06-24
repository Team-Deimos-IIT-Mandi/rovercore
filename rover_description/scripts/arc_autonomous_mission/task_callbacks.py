import yasmin
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED


def find_highest_point(blackboard: Blackboard) -> str:
    yasmin.YASMIN_LOG_INFO(
        "Scanning antenna area for highest elevation point..."
    )
    yasmin.YASMIN_LOG_INFO("Highest point found at coordinates (X: 7.2, Y: -8.5)")
    return SUCCEED


def search_dark_rocks(blackboard: Blackboard) -> str:
    yasmin.YASMIN_LOG_INFO(
        "Searching for dark rocks within 10m diameter of Shackleton crater..."
    )
    yasmin.YASMIN_LOG_INFO("Dark rock search complete — 3 targets identified")
    return SUCCEED


def measure_lava_tube(blackboard: Blackboard) -> str:
    yasmin.YASMIN_LOG_INFO("Measuring length of lava tube...")
    yasmin.YASMIN_LOG_INFO("Lava tube length: 12.4 metres")
    return SUCCEED


TASK_CALLBACKS = {
    "FIND_HIGHEST_POINT": find_highest_point,
    "SEARCH_DARK_ROCKS": search_dark_rocks,
    "MEASURE_LAVA_TUBE": measure_lava_tube,
}
