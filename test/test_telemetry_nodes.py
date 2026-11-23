"""Launch test for the telemetry nodes the web dashboard relies on.

* nav_status_node: Nav2 goal status/feedback -> cmeresearch_msgs/NavStatus
* map_pose_node:   map->base_link TF        -> /robot_pose
* system_stats_node: publishes plausible SystemStats
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
from action_msgs.msg import GoalStatus, GoalStatusArray
from builtin_interfaces.msg import Duration, Time
from cmeresearch_msgs.msg import NavStatus, SystemStats
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

LATCHED = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node
    return launch.LaunchDescription([
        node(package='cmeresearch_robot_state', executable='nav_status_node'),
        node(package='cmeresearch_robot_state', executable='map_pose_node',
             parameters=[{'publish_rate': 20.0}]),
        node(package='cmeresearch_robot_state', executable='system_stats_node',
             parameters=[{'publish_rate_hz': 5.0}]),
        launch_testing.actions.ReadyToTest(),
    ])


def goal(status, stamp_sec):
    g = GoalStatus()
    g.status = status
    g.goal_info.stamp = Time(sec=stamp_sec)
    return g


class TestTelemetryNodes(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('telemetry_test')
        cls.lock = threading.Lock()
        cls.nav, cls.poses, cls.stats = [], [], []
        cls.node.create_subscription(NavStatus, '/nav_status', cls._rec(cls.nav), LATCHED)
        cls.node.create_subscription(PoseStamped, '/robot_pose', cls._rec(cls.poses), 10)
        cls.node.create_subscription(SystemStats, '/system_stats', cls._rec(cls.stats), LATCHED)
        cls.status_pub = cls.node.create_publisher(
            GoalStatusArray, '/navigate_to_pose/_action/status', 10)
        cls.feedback_pub = cls.node.create_publisher(
            NavigateToPose.Impl.FeedbackMessage, '/navigate_to_pose/_action/feedback', 10)
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
    def _rec(cls, store):
        def cb(msg):
            with cls.lock:
                store.append(msg)
        return cb

    def wait_for(self, predicate, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.lock:
                if predicate():
                    return True
            time.sleep(0.05)
        return False

    def last_nav(self):
        return self.nav[-1] if self.nav else None

    def publish_status_until(self, goals, expected):
        msg = GoalStatusArray(status_list=goals)

        def reached():
            n = self.last_nav()
            return n is not None and n.status == expected

        end = time.monotonic() + 10.0
        while time.monotonic() < end:
            self.status_pub.publish(msg)
            if self.wait_for(reached, timeout=0.2):
                return self.last_nav()
        self.fail(f'nav status never became {expected!r}, last: {self.last_nav()}')

    # --- nav_status_node --------------------------------------------------------

    def test_nav_status_sequence(self):
        self.assertTrue(self.wait_for(lambda: bool(self.nav)), 'no latched /nav_status')
        self.assertEqual(self.nav[0].status, 'idle')

        self.publish_status_until([goal(GoalStatus.STATUS_EXECUTING, 1)], 'navigating')

        fb = NavigateToPose.Impl.FeedbackMessage()
        fb.feedback.distance_remaining = 3.5
        fb.feedback.estimated_time_remaining = Duration(sec=12, nanosec=500_000_000)
        fb.feedback.navigation_time = Duration(sec=4)
        fb.feedback.number_of_recoveries = 2

        def progress():
            n = self.last_nav()
            return n is not None and abs(n.distance_remaining - 3.5) < 1e-6

        end = time.monotonic() + 10.0
        while time.monotonic() < end and not self.wait_for(progress, 0.2):
            self.feedback_pub.publish(fb)
        n = self.last_nav()
        self.assertEqual(n.status, 'navigating')
        self.assertAlmostEqual(n.estimated_time_remaining, 12.5, places=4)
        self.assertAlmostEqual(n.navigation_time, 4.0, places=4)
        self.assertEqual(n.number_of_recoveries, 2)

        # Terminal: progress fields are zeroed.
        done = self.publish_status_until([goal(GoalStatus.STATUS_SUCCEEDED, 2)], 'succeeded')
        self.assertEqual(done.distance_remaining, 0.0)
        self.assertEqual(done.number_of_recoveries, 0)

        # The newest terminal goal decides, regardless of list order.
        self.publish_status_until(
            [goal(GoalStatus.STATUS_ABORTED, 10), goal(GoalStatus.STATUS_SUCCEEDED, 5)],
            'aborted')
        # An active goal wins over terminal ones.
        self.publish_status_until(
            [goal(GoalStatus.STATUS_CANCELED, 20), goal(GoalStatus.STATUS_ACCEPTED, 3)],
            'navigating')
        self.publish_status_until([goal(GoalStatus.STATUS_CANCELING, 21)], 'canceling')
        self.publish_status_until([], 'idle')

    # --- map_pose_node -------------------------------------------------------------

    def test_map_pose_follows_tf(self):
        time.sleep(1.0)
        with self.lock:
            self.assertEqual(self.poses, [], 'published a pose without map->base_link TF')

        tf = TransformStamped()
        tf.header.stamp = self.node.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = 1.5
        tf.transform.translation.y = -2.0
        tf.transform.rotation.z = 0.7071068
        tf.transform.rotation.w = 0.7071068
        broadcaster = StaticTransformBroadcaster(self.node)
        broadcaster.sendTransform(tf)

        self.assertTrue(self.wait_for(lambda: bool(self.poses)), 'no /robot_pose after TF')
        with self.lock:
            p = self.poses[-1]
        self.assertEqual(p.header.frame_id, 'map')
        self.assertAlmostEqual(p.pose.position.x, 1.5)
        self.assertAlmostEqual(p.pose.position.y, -2.0)
        self.assertAlmostEqual(p.pose.orientation.z, 0.7071068)
        self.assertAlmostEqual(p.pose.orientation.w, 0.7071068)

    # --- system_stats_node ---------------------------------------------------------

    def test_system_stats_are_plausible(self):
        self.assertTrue(self.wait_for(lambda: len(self.stats) >= 2), 'no /system_stats')
        with self.lock:
            s = self.stats[-1]
        self.assertTrue(0.0 <= s.cpu_percent <= 100.0, s.cpu_percent)
        self.assertGreater(s.memory_total_mb, 0)
        self.assertLessEqual(s.memory_used_mb, s.memory_total_mb)
        self.assertTrue(0.0 <= s.memory_percent <= 100.0)
        self.assertGreater(s.disk_total_mb, 0)
        self.assertLessEqual(s.disk_used_mb, s.disk_total_mb)
        self.assertTrue(0.0 <= s.disk_percent <= 100.0)
        self.assertTrue(s.kernel_release, 'kernel_release empty')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_nodes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, [0, -2, -15])
