"""
Central Mission Manager Node for the Rover.
Sequence:
  1. BOOTING           -> System startup
  2. DOME_EXIT         -> Handled by airlock_exit_node.py (Vision + Blind Push)
  3. ASTRONAUT_SEARCH  -> Handled by astronaut_searcher.py (odom target drive)
  4. FUEL_TRAIL_FOLLOW -> Handled by fuel_trail_follower.py (HSV Line Tracker)
  5. OXYGEN_TANK_NAV   -> Handled by oxygen_tank_navigator.py (odom target drive)
  6. DOME_RETURN       -> Handled by dome_return_navigator.py (odom target drive)
  7. SEQUENCE_COMPLETE -> End of integrated track timeline
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        
        self.VALID_STATES = {
            'BOOTING', 
            'DOME_EXIT', 
            'ASTRONAUT_SEARCH',
            'OXYGEN_TANK_NAV',
            'DOME_RETURN',
            # 'ALIGN_AND_MOVE', 
            # 'astronaut', 
            'FUEL_TRAIL_FOLLOW', 
            'SEQUENCE_COMPLETE', 
            'EMERGENCY_STOP'
        }
        self.current_state = 'BOOTING'
        
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.state_pub = self.create_publisher(String, 'rover/mission_state', qos_profile)
        
        # Handshake endpoints
        self.complete_exit_srv = self.create_service(Trigger, 'mission/complete_dome_exit', self.complete_dome_exit_callback)
        self.complete_astronaut_search_srv = self.create_service(Trigger, 'mission/complete_astronaut_search', self.complete_astronaut_search_callback)
        self.complete_fuel_trail_srv = self.create_service(Trigger, 'mission/complete_fuel_trail', self.complete_fuel_trail_callback)
        self.complete_oxygen_tank_srv = self.create_service(Trigger, 'mission/complete_oxygen_tank_nav', self.complete_oxygen_tank_callback)
        self.complete_dome_return_srv = self.create_service(Trigger, 'mission/complete_dome_return', self.complete_dome_return_callback)
        # self.complete_move_srv = self.create_service(Trigger, 'mission/complete_alignment_move', self.complete_alignment_callback)
        # self.complete_final_turn_srv = self.create_service(Trigger, 'mission/complete_final_turn', self.complete_final_turn_callback)
        self.estop_srv = self.create_service(Trigger, 'mission/emergency_stop', self.emergency_stop_callback)
        
        self.publish_state()
        self.get_logger().info("Mission Manager Initialized. System Booting...")

    def publish_state(self):
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)
        self.get_logger().info(f"[FSM STATE UPDATE] ==> {self.current_state}")

    def change_state(self, new_state):
        if new_state in self.VALID_STATES:
            self.current_state = new_state
            self.publish_state()
        else:
            self.get_logger().error(f"Rejected transition! State '{new_state}' is invalid.")

    def complete_dome_exit_callback(self, request, response):
        if self.current_state == 'DOME_EXIT':
            self.get_logger().info("AirlockExitNode finished. Moving to ASTRONAUT_SEARCH.")
            self.change_state('ASTRONAUT_SEARCH')
            response.success = True
            response.message = "Global state shifted from 'DOME_EXIT' to 'ASTRONAUT_SEARCH'."
        else:
            response.success = False
        return response

    def complete_astronaut_search_callback(self, request, response):
        if self.current_state == 'ASTRONAUT_SEARCH':
            self.get_logger().info("Astronaut Searcher finished. Moving to FUEL_TRAIL_FOLLOW.")
            self.change_state('FUEL_TRAIL_FOLLOW')
            response.success = True
            response.message = "Global state shifted from 'ASTRONAUT_SEARCH' to 'FUEL_TRAIL_FOLLOW'."
        else:
            response.success = False
        return response

    def complete_fuel_trail_callback(self, request, response):
        if self.current_state == 'FUEL_TRAIL_FOLLOW':
            self.get_logger().info("Fuel Trail Follower finished. Moving to OXYGEN_TANK_NAV.")
            self.change_state('OXYGEN_TANK_NAV')
            response.success = True
            response.message = "Global state shifted from 'FUEL_TRAIL_FOLLOW' to 'OXYGEN_TANK_NAV'."
        else:
            response.success = False
        return response

    def complete_oxygen_tank_callback(self, request, response):
        if self.current_state == 'OXYGEN_TANK_NAV':
            self.get_logger().info("Oxygen Tank Navigator finished. Returning to DOME_RETURN.")
            self.change_state('DOME_RETURN')
            response.success = True
            response.message = "Global state shifted from 'OXYGEN_TANK_NAV' to 'DOME_RETURN'."
        else:
            response.success = False
        return response

    def complete_dome_return_callback(self, request, response):
        if self.current_state == 'DOME_RETURN':
            self.get_logger().info("Dome Return Navigator finished. Sequence complete.")
            self.change_state('SEQUENCE_COMPLETE')
            response.success = True
            response.message = "Global state shifted from 'DOME_RETURN' to 'SEQUENCE_COMPLETE'."
        else:
            response.success = False
        return response

    # def complete_alignment_callback(self, request, response):
    #     if self.current_state == 'ALIGN_AND_MOVE':
    #         self.get_logger().info("AlignAndMoveNode finished. Moving to astronaut phase.")
    #         self.change_state('astronaut')
    #         response.success = True
    #     else:
    #         response.success = False
    #     return response

    # def complete_final_turn_callback(self, request, response):
    #     """Modified: Shifting into trail follower instead of terminating directly"""
    #     if self.current_state == 'astronaut':
    #         self.get_logger().info("110-Degree Turn Complete! Activating Reman's Fuel Trail Follower Node.")
            
    #         # Switch state to hand off control to Reman's script
    #         self.change_state('FUEL_TRAIL_FOLLOW')
            
    #         response.success = True
    #         response.message = "Global state shifted from 'astronaut' to 'FUEL_TRAIL_FOLLOW'."
    #     else:
    #         response.success = False
    #     return response

    def emergency_stop_callback(self, request, response):
        self.get_logger().warn("!!! EMERGENCY STOP ACTIVE !!!")
        self.change_state('EMERGENCY_STOP')
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    node.change_state('DOME_EXIT')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()