#!/usr/bin/env python3
import threading
from evdev import InputDevice, ecodes
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from geometry_msgs.msg import Point

class OpticalSensorNode(LifecycleNode):
    def __init__(self):
        super().__init__('optical_sensor_node')
        self.device_path = '/dev/input/event0'
        self.device = None
        self.publisher_ = None
        self.thread = None
        self._stop_event = threading.Event()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring...')
        self.publisher_ = self.create_lifecycle_publisher(Point, '/optical', 10)

        try:
            self.device = InputDevice(self.device_path)
            self.get_logger().info(
                f"Opened device: {self.device.name} ({self.device.path})"
            )
        except Exception as e:
            self.get_logger().error(
                f"Failed to open device {self.device_path}: {e}"
            )
            return TransitionCallbackReturn.FAILURE

        return super().on_configure(state)

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating...')
        self._stop_event.clear()
        self.thread = threading.Thread(
            target=self.read_device_events, daemon=True
        )
        self.thread.start()
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating...')
        self._stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up...')
        self._stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        if self.device is not None:
            self.device.close()
            self.device = None
        if self.publisher_ is not None:
            self.destroy_publisher(self.publisher_)
            self.publisher_ = None
        return super().on_cleanup(state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutdown requested.')
        return super().on_shutdown(state)

    def read_device_events(self):
        if not self.device:
            return
        for event in self.device.read_loop():
            if self._stop_event.is_set():
                break
            if event.type == ecodes.EV_REL:
                point = Point()
                if event.code == ecodes.REL_X:
                    point.x = float(event.value)
                else:
                    point.x = 0.0
                if event.code == ecodes.REL_Y:
                    point.y = float(event.value)
                else:
                    point.y = 0.0
                point.z = 0.0
                self.publisher_.publish(point)
        self.get_logger().info('Event-reading thread stopped.')

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    node = OpticalSensorNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
