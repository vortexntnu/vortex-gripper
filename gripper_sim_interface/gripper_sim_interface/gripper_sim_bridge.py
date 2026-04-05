#!/usr/bin/env python3
"""
Gripper Sim Bridge
Rewires the vortex gripper pipeline to use Stonefish/Nautilus simulator topics.

Joystick reference (joystick_interface_auv, nautilus branch):
  /nautilus/gripper_servos <- JointState
      name     = [arm_joint, finger_joint1, finger_joint2]
      velocity = [roll,    grip,          grip          ]
      frame_id = "base_link"

This bridge mirrors that format exactly, driven by the gripper pipeline:
  /vortex/gripper/control (GripperStateVelocityCommand)
      roll_dot  -> arm_joint velocity       (≡ rotate)
      pinch_dot -> finger_joint1/2 velocity (≡ grip)

State feedback — raw radians, no normalisation:
  /nautilus/servo_state (JointState) -> /vortex/gripper/state (GripperState)
  Finger avg: -0.33 rad = fully open, ~0.0 rad = fully closed.

NOTE: Keep the AUV in AUTONOMOUS mode during gripper pipeline tests to avoid
a topic conflict — the joystick also publishes to /nautilus/gripper_servos in
MANUAL and REFERENCE modes.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from vortex_msgs.msg import GripperState, GripperStateVelocityCommand

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)


class GripperSimBridge(Node):
    def __init__(self):
        super().__init__('gripper_sim_bridge')

        self.create_subscription(
            JointState,
            '/nautilus/servo_state',
            self.on_servo_state,
            BEST_EFFORT_QOS,
        )
        self.create_subscription(
            GripperStateVelocityCommand,
            '/vortex/gripper/control',
            self.on_velocity_command,
            BEST_EFFORT_QOS,
        )

        self.state_pub = self.create_publisher(
            GripperState, '/vortex/gripper/state', RELIABLE_QOS
        )
        self.cmd_pub = self.create_publisher(
            JointState, '/nautilus/gripper_servos', BEST_EFFORT_QOS
        )

        self.get_logger().info('GripperSimBridge: online')
        self.get_logger().info('  IN  /nautilus/servo_state    -> /vortex/gripper/state')
        self.get_logger().info('  OUT /vortex/gripper/control  -> /nautilus/gripper_servos')
        self.get_logger().warn(
            'Keep AUV in AUTONOMOUS mode to avoid joystick conflict on /nautilus/gripper_servos'
        )
    
    def on_servo_state(self, msg: JointState):
        try:
            idx_arm = msg.name.index('nautilus/arm_joint')
            idx_f1  = msg.name.index('nautilus/finger_joint1')
            idx_f2  = msg.name.index('nautilus/finger_joint2')
        except ValueError as e:
            self.get_logger().warn(f'Joint name missing: {e}', throttle_duration_sec=5.0)
            return

        state              = GripperState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.roll         = msg.position[idx_arm]
        state.pinch        = (msg.position[idx_f1] + msg.position[idx_f2]) / 2.0

        self.state_pub.publish(state)

    def on_velocity_command(self, msg: GripperStateVelocityCommand):
        cmd                  = JointState()
        cmd.header.stamp     = self.get_clock().now().to_msg()
        cmd.header.frame_id  = 'base_link'
        cmd.name             = [
            'nautilus/arm_joint',
            'nautilus/finger_joint1',
            'nautilus/finger_joint2',
        ]

        cmd.position         = []
        cmd.velocity         = [msg.roll_velocity, msg.pinch_velocity, msg.pinch_velocity]
        cmd.effort           = []
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = GripperSimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
