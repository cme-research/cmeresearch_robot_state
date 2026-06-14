import threading
import time
import unittest

from cmeresearch_msgs.msg import RobotState
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='cmeresearch_robot_state',
            executable='sm_robot_node',
            name='sm_robot',
            output='screen',
            # The test fixture never spins up tinkerforge driver mocks, so the
            # SM would block in Initializing for the default 30 s timeout. Drop
            # the timeout to 0.5 s so the SM falls back to idle quickly and the
            # rest of the state-transition tests can run.
            parameters=[{'init_timeout_sec': 0.5}],
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestStateTransitions(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_state_transitions')
        cls.states = []
        cls.lock = threading.Lock()

        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        cls.sub = cls.node.create_subscription(
            RobotState, '/robot_state',
            lambda msg: cls._on_state(msg), qos)
        cls.pub = cls.node.create_publisher(String, '/robot_cmd', 10)

        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_state(cls, msg):
        with cls.lock:
            cls.states.append(msg.state)

    def _wait_for_state(self, expected, timeout=10.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.lock:
                if expected in self.states:
                    return True
            time.sleep(0.05)
        return False

    def _send_cmd(self, cmd):
        msg = String()
        msg.data = cmd
        self.pub.publish(msg)

    def test_01_startup_reaches_idle(self):
        """Startup auto-transitions to idle state."""
        self.assertTrue(
            self._wait_for_state('idle'),
            'Never reached idle state on startup')

    def test_02_start_mission_idle_to_moving(self):
        """Start_mission command transitions idle to moving."""
        self._wait_for_state('idle')
        self._send_cmd('start_mission')
        self.assertTrue(
            self._wait_for_state('moving'),
            'Never reached moving state after start_mission')

    def test_03_emergency_stop_from_moving(self):
        """Emergency_stop command transitions any state to emergency_stop."""
        self._wait_for_state('moving')
        self._send_cmd('emergency_stop')
        self.assertTrue(
            self._wait_for_state('emergency_stop'),
            'Never reached emergency_stop state')

    def test_04_reset_from_emergency_stop(self):
        """Reset command transitions emergency_stop back to idle."""
        self._wait_for_state('emergency_stop')
        with self.lock:
            self.states.clear()
        self._send_cmd('reset')
        self.assertTrue(
            self._wait_for_state('idle'),
            'Never reached idle after reset')
