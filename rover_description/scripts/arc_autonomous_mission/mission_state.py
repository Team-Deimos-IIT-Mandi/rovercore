import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

class RoverSensorNode(LifecycleNode):
    def __init__(self):
        # Initializes the node in the 'UNCONFIGURED' state
        super().__init__('rover_sensor_node')
        self.timer = None
        self.get_logger().info("Sensor Node Born. State: UNCONFIGURED")

    def on_configure(self, state):
        """Called when transitioning Unconfigured -> Inactive."""
        self.get_logger().info("Configuring sensors and allocating memory...")
        # Put your hardware initializations here (e.g., opening a camera port)
        
        # Setting up a timer, but NOT starting it yet
        self.timer = self.create_timer(1.0, self.timer_callback)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        """Called when transitioning Inactive -> Active."""
        self.get_logger().info("🟢 Sensor Node ACTIVE. Beginning task execution...")
        # Active state means the node is allowed to process and publish data
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """Called when transitioning Active -> Inactive."""
        self.get_logger().info("🟡 Sensor Node INACTIVE. Powering down sensor to save battery...")
        # Stop processing data, but don't destroy the configuration
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        """Called when transitioning Inactive -> Unconfigured."""
        self.get_logger().info("🧹 Cleaning up resources and freeing memory...")
        if self.timer:
            self.timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        # Guard clause: Lifecycle nodes should only run logic if fully ACTIVE
        if self.lifecycle_state.label != 'active':
            return
        self.get_logger().info("Reading sensor data stream... [X: 0.23, Y: 0.89]")

def main(args=None):
    rclpy.init(args=args)
    node = RoverSensorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()