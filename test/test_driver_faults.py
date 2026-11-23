"""Driver faults reported on the stepper `state` topics.

* `disconnected` after start-up triggers an emergency stop (from idle and moving).
* A reset waits for the faulty driver: the init-timeout backstop must not
  force idle while a driver explicitly reports a non-ready state.
* Once the driver reports `initialized` again, the robot returns to idle.
"""

import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from cmeresearch_msgs.msg import RobotState

DRIVERS = ['/fault_test/front_left/state', '/fault_test/front_right/state']
INIT_TIMEOUT = 2.0
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='cmeresearch_robot_state',
            executable='sm_robot_node',
            name='sm_robot',
            output='screen',
            parameters=[{'driver_topics': DRIVERS, 'init_timeout_sec': INIT_TIMEOUT}],
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestDriverFaults(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_driver_faults')
        cls.lock = threading.Lock()
        cls.states = []
        cls.node.create_subscription(
            RobotState, '/robot_state', cls._on_state,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        cls.cmd = cls.node.create_publisher(String, '/robot_cmd', 10)
        cls.driver_pubs = {t: cls.node.create_publisher(String, t, LATCHED) for t in DRIVERS}
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_state(cls, msg):
        with cls.lock:
            cls.states.append(msg.state)

    def last_state(self):
        with self.lock:
            return self.states[-1] if self.states else None

    def wait_for(self, state, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.last_state() == state:
                return True
            time.sleep(0.05)
        return False

    def driver(self, index, state):
        self.driver_pubs[DRIVERS[index]].publish(String(data=state))

    def command(self, cmd):
        self.cmd.publish(String(data=cmd))

    def test_disconnected_driver_triggers_emergency_stop(self):
        # Start-up: both drivers ready -> idle.
        time.sleep(1.0)   # let the latched subscriptions match
        self.driver(0, 'initialized')
        self.driver(1, 'initialized')
        self.assertTrue(self.wait_for('idle'), f'never idle, last {self.last_state()}')

        # Moving + a wheel driver loses its bricklet -> emergency stop.
        self.command('start_mission')
        self.assertTrue(self.wait_for('moving'))
        self.driver(0, 'disconnected')
        self.assertTrue(self.wait_for('emergency_stop'),
                        f'no emergency stop, last {self.last_state()}')

        # Reset while the driver is still gone: stays initializing, even past
        # the init timeout (the backstop must not force idle).
        self.command('reset')
        self.assertTrue(self.wait_for('initializing'))
        time.sleep(INIT_TIMEOUT + 1.5)
        self.assertEqual(self.last_state(), 'initializing',
                         'robot became drivable with a disconnected wheel')

        # Driver back -> idle.
        self.driver(0, 'initialized')
        self.assertTrue(self.wait_for('idle'), f'not idle after reconnect, last {self.last_state()}')

        # Disconnect while idle also stops.
        self.driver(1, 'disconnected')
        self.assertTrue(self.wait_for('emergency_stop'))
