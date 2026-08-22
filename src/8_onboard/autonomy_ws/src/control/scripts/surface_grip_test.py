#!/usr/bin/env python3
"""
Drive a constant-steering circle at stepped speeds to find the grip limit.

The joystick is scaled to 1.0 m/s, which is far below where the tyres let go,
so the surface limit cannot be reached by hand.  This publishes the drive
command directly instead, and holds each speed long enough for the yaw rate to
settle.  Record the run with `ros2 bag record -a` and feed it to
analyze_grip_bag.py.

Operation:

  1. Run the vehicle bringup only.  Do NOT start run_autonomy.sh -- minjae_pp
     publishes a zero command to the same topic at 50 Hz while disabled, and
     the two publishers would fight.  This script warns if it sees one.
  2. Put the car on a stand and verify the kill switch before wheels-down:
     with the deadman released the wheels should follow this script, and
     holding the deadman with the sticks centred should stop them.  That is
     the abort action for the whole run.
  3. Wheels down, hold a clear circle, release the deadman, and let the steps
     run.  Grab the deadman to abort at any point.
  4. --brake-from adds a straight-line stop after the steps, which is the only
     way to measure speed_profile_deceleration.  It needs a straight runway,
     not the circle, so it prints the distance it will use before starting.

Note that throttle_interpolator and safety_node are both commented out of
bringup_launch.py on this car, so nothing smooths the command and nothing
stops the car automatically.  The deadman is the only brake.

The deadman is button 4 (L1) on this car -- every non-zero /teleop command in
the 0809_test_6 mapping bag has it held, and no other button is ever pressed.
Ctrl+C publishes a zero command before exiting.

Usage:
  python3 surface_grip_test.py --steering 0.30 --speeds 0.8,1.2,1.6,2.0,2.4
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped


class SurfaceGripTest(Node):

    def __init__(self, args):
        super().__init__('surface_grip_test')
        self.steering = args.steering
        self.speeds = args.speeds
        self.dwell = args.dwell
        self.rate = args.rate
        self.brake_from = args.brake_from
        self.brake_hold = args.brake_hold
        self.publisher = self.create_publisher(
            AckermannDriveStamped, args.topic, 10)
        others = self.count_publishers(args.topic) - 1
        if others > 0:
            self.get_logger().error(
                '%d other publisher(s) on %s -- stop the autonomy stack '
                'first, or the commands will interleave.'
                % (others, args.topic))
        self.get_logger().info(
            'topic=%s steering=%.3f rad (%.1f deg) speeds=%s dwell=%.1fs'
            % (args.topic, self.steering, math.degrees(self.steering),
               self.speeds, self.dwell))

    def publish(self, speed):
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.drive.speed = float(speed)
        message.drive.steering_angle = float(self.steering)
        self.publisher.publish(message)

    def hold(self, speed, seconds):
        """Publish one command for the given duration, then return."""
        period = 1.0 / self.rate
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if not rclpy.ok():
                return False
            self.publish(speed)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        return True

    def run(self):
        for remaining in range(3, 0, -1):
            self.get_logger().warn('starting in %d ...' % remaining)
            if not self.hold(0.0, 1.0):
                return
        for speed in self.speeds:
            self.get_logger().info('step %.2f m/s' % speed)
            if not self.hold(speed, self.dwell):
                return
        if self.brake_from > 0.0:
            self.brake_test()
        self.get_logger().info('done, stopping')

    def brake_test(self):
        """Straight-line stop, held long enough to record the whole ramp."""
        self.steering = 0.0
        self.get_logger().warn(
            'BRAKE TEST: straighten up, %.2f m/s for %.1fs then full stop'
            % (self.brake_from, self.brake_hold))
        if not self.hold(0.0, 2.0):
            return
        if not self.hold(self.brake_from, self.brake_hold):
            return
        # Keep publishing zero: the deceleration ramp is what gets measured,
        # and letting the node exit here would leave the mux to time out
        # instead of showing a commanded stop.
        self.hold(0.0, 3.0)


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/auto',
                        help='drive topic (/auto on the car, /drive in sim)')
    parser.add_argument('--steering', type=float, default=0.30,
                        help='constant steering angle in rad')
    parser.add_argument('--speeds', default='0.8,1.2,1.6,2.0,2.4',
                        help='comma separated speed steps in m/s')
    parser.add_argument('--dwell', type=float, default=5.0,
                        help='seconds to hold each speed step')
    parser.add_argument('--rate', type=float, default=50.0,
                        help='publish rate in Hz')
    parser.add_argument('--brake-from', type=float, default=0.0,
                        help='straight-line stop from this speed; 0 skips it')
    parser.add_argument('--brake-hold', type=float, default=1.5,
                        help='seconds at speed before the stop')
    args = parser.parse_args(argv)
    args.speeds = [float(value) for value in args.speeds.split(',')]
    if abs(args.steering) > 0.3785:
        parser.error(
            'steering beyond 0.3785 rad saturates the servo on one side')
    if any(speed <= 0.0 for speed in args.speeds):
        parser.error('speed steps must be positive')
    if args.brake_from > 0.0:
        # Assume a pessimistic 2.0 m/s^2 so the printed runway is not short.
        runway = (args.brake_from * args.brake_hold
                  + args.brake_from ** 2 / (2.0 * 2.0))
        print('brake test needs about %.1f m of straight runway' % runway)
    return args


def main():
    args = parse_args(sys.argv[1:])
    rclpy.init()
    node = SurfaceGripTest(args)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn('interrupted')
    finally:
        for _ in range(50):
            node.publish(0.0)
            time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
