#!/usr/bin/env python3
# FEATURE: Native Player server and Python audio replace all ROS runtime dependencies.
# SPEC: The supported build and runtime contain no ROS launch, SLAM or wake-word components.
# IMPLEMENTATION: Native workspace, one lidar web bundle, and a standalone Python audio package.
# RESULT: IMPLEMENTED — obsolete entry points and middleware dependencies are absent.
# STATUS: DONE
from pathlib import Path
import re
import tomllib
import unittest

ROOT = Path(__file__).resolve().parents[1]


class NativeArchitectureTests(unittest.TestCase):
    def test_default_workspace_builds_the_native_server(self):
        workspace = tomllib.loads((ROOT / 'crates/Cargo.toml').read_text())['workspace']
        self.assertIn('robot-server', workspace['members'])
        self.assertNotIn('player_to_ros2', workspace['members'])
        self.assertIn('robot-server', workspace.get('default-members', workspace['members']))
        package = tomllib.loads((ROOT / 'crates/robot-server/Cargo.toml').read_text())
        self.assertEqual(package['package']['name'], 'robot-server')

    def test_build_has_no_ros_message_or_client_dependencies(self):
        forbidden = {'rclrs', 'ros-env', 'ros_env', 'tf2_msgs', 'sensor_msgs', 'nav_msgs',
                     'rosidl_runtime_rs', 'ament_rs'}
        for manifest in ('crates/Cargo.lock', 'crates/player-bridge/Cargo.toml',
                         'crates/mask-owner/Cargo.toml', 'crates/robot-server/Cargo.toml'):
            path = ROOT / manifest
            self.assertTrue(path.is_file(), manifest)
            content = path.read_text()
            for name in forbidden:
                self.assertNotRegex(content, rf'(?m)^(?:name\s*=\s*"{re.escape(name)}"|{re.escape(name)}\s*=)', manifest)

    def test_old_navigation_and_python_panel_entry_points_are_removed(self):
        for relative in ('crates/player_to_ros2/Cargo.toml', 'ros2/web_panel.py', 'ros2/panel.html',
                         'ros2/odom_tf_replay.py', 'ros2/route_check.py', 'ros2/sensor_overlay.py',
                         'scripts/run-rviz.sh', 'scripts/run-sim.sh', 'scripts/run-mapping.sh',
                         'scripts/slam-offline.sh', 'scripts/slam-from-bag.sh', 'scripts/amcl-replay.sh',
                         'scripts/nav2-replay.sh', 'scripts/sim-route.sh'):
            self.assertFalse((ROOT / relative).exists(), relative)
        self.assertTrue((ROOT / 'web/panel.html').is_file())

    def test_python_audio_does_not_depend_on_ros_or_wake_modules(self):
        package = ROOT / 'python/robot_audio'
        self.assertTrue((package / 'runtime.py').is_file())
        for name in ('ros_node.py', 'wake.py', 'training.py'):
            self.assertFalse((package / name).exists(), name)
        forbidden = re.compile(r'(?m)^\s*(?:from|import)\s+(?:rclpy|rosidl|robot_audio_interfaces|openwakeword)(?:\b|\.)')
        for path in [*(package.glob('*.py')), *(ROOT / 'bin/audio').glob('*.py')]:
            self.assertNotRegex(path.read_text(), forbidden, str(path.relative_to(ROOT)))
        self.assertFalse((ROOT / 'ros2/robot_audio/runtime.py').exists())
        self.assertFalse((ROOT / 'ros2/robot_audio_interfaces/package.xml').exists())

    def test_cli_and_frontend_do_not_launch_ros_or_offer_mapping(self):
        cli = (ROOT / 'bin/robot').read_text()
        self.assertNotRegex(cli, r'(?m)^\s*(?:source_ros\b|source .*?/opt/ros|ros2\s+(?:run|launch))')
        ui = (ROOT / 'tools/panel-ui/src/App.jsx').read_text()
        self.assertNotIn('useOccupancyMap', ui)
        self.assertNotIn('map_pose', ui)
        self.assertFalse((ROOT / 'tools/panel-ui/src/useOccupancyMap.js').exists())


if __name__ == '__main__':
    unittest.main()
