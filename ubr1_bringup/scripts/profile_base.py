#!/usr/bin/env python3

# Copyright 2023, Michael Ferguson
# Copyright 2013-2014, Unbounded Robotics, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
This script is used to profile the robot base. It commands a particular
velocity to the base_controller and then plots the actual outputs.
"""

import argparse
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

# visualization
from pylab import *

class BaseTest(Node):

    ## x is m/s, accel_x is m/s^2
    ## r is rad/s, accel_r is rad/s^2
    def __init__(self, x, r, plot_effort, plot_velocity, accel_x = 0.5, accel_r = 2.97044, rate = 100.0):
        super().__init__("profile_base")

        # Create a publisher to command the base
        self.pub = self.create_publisher(Twist, "base_controller/command", 5)

        # Rate to publish and run loop at
        self.rate = rate

        # Set acceleration
        self.x_accel = accel_x / rate
        self.r_accel = accel_r / rate

        self.x = x
        self.r = r

        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0

        self.data = list()
        self.error = list()
        self.odom_sub = self.create_subscription(Odometry, "base_controller/odom", self.odomCb, 10)

        self.effort = list()
        self.velocity = list()
        
        # Should we plot effort?
        self.plot_effort = plot_effort
        self.plot_velocity = plot_velocity
        if plot_effort or plot_velocity:
            self.js_sub = self.create_subscription(JointState, "joint_states", self.stateCb, 10)

    def odomCb(self, msg):
        if abs(self.vel.linear.x) > 0 or abs(self.vel.angular.z) > 0:
            self.data.append([self.vel.linear.x, self.vel.angular.z, msg.twist.twist.linear.x, msg.twist.twist.angular.z])
            self.error.append([msg.twist.twist.linear.x - self.vel.linear.x, msg.twist.twist.angular.z - self.vel.angular.z])

    def stateCb(self, msg):
        if abs(self.vel.linear.x) > 0 or abs(self.vel.angular.z) > 0:
            # assume base_l, base_r are first and second element
            self.effort.append([msg.effort[0], msg.effort[1]])
            self.velocity.append([msg.velocity[0], msg.velocity[1]])

    def ramp(self):
        r = self.create_rate(self.rate) # match odometry for plotting
        while abs(self.vel.linear.x - self.x) > self.x_accel or abs(self.vel.angular.z - self.r) > self.r_accel:

            if self.x - self.vel.linear.x > self.x_accel:
                self.vel.linear.x += self.x_accel
            elif self.vel.linear.x - self.x > self.x_accel:
                self.vel.linear.x -= self.x_accel

            if self.r - self.vel.angular.z > self.r_accel:
                self.vel.angular.z += self.r_accel
            elif self.vel.angular.z - self.r > self.r_accel:
                self.vel.angular.z -= self.r_accel
            
            self.pub.publish(self.vel)
            r.sleep()

    def run(self, constant_duration):
        r = self.create_rate(self.rate) # match odometry for plotting

        self.get_logger().info("Ramping up to %f, %f" % (self.x, self.r))
        self.ramp()

        # run forward for constant_duration seconds
        self.get_logger().info("Constant velocity")
        for i in range(int(constant_duration * self.rate)):
            self.pub.publish(self.vel)
            r.sleep()

        # ramp speed down
        self.get_logger().info("Ramping to a stop")
        self.x = 0.0
        self.r = 0.0
        self.ramp()
        self.vel = Twist()
        self.get_logger().info("Done")

    def plot(self):
        x_err = [abs(e[0]) for e in self.error]
        r_err = [abs(e[1]) for e in self.error]
        avg_x_err = sum(x_err)/len(self.error)
        avg_r_err = sum(r_err)/len(self.error)
        max_x_err = max(x_err)
        max_r_err = max(r_err)

        x_data = [abs(x[2]) for x in self.data]
        print("  Max Vel: %f" % max(x_data))

        print("Linear error")
        print("  Mean: %f" % (sum(x_err)/len(self.error)))
        print("  Max: %f" % max(x_err))
        print("Angular error")
        print("  Mean: %f" % (sum(r_err)/len(self.error)))
        print("  Max: %f" % max(r_err))

        time = [i/self.rate for i in range(len(self.data))]
        size_x = 2
        if self.plot_effort:
            size_x += 1
        if self.plot_velocity:
            size_x += 1

        subplot(size_x, 1, 1)
        plot(time, self.data)
        legend(('Desired Linear Velocity', 'Desired Angular Velocity', 'Actual Linear Velocity', 'Actual Angular Velocity'))
        subplot(size_x, 1, 2)
        plot(time, self.error)
        legend(('Linear Error', 'Angular Error'))
        if self.plot_effort:
            subplot(size_x, 1, 3)
            plot(self.effort)
            legend(('Left', 'Right'))
        if self.plot_velocity:
            if self.plot_effort:
                subplot(size_x, 1, 4)
            else:                
                subplot(size_x, 1, 3)
            plot(self.velocity)
            legend(('Left', 'Right'))
        show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-x", help="Linear velocity, in m/s", type=float, default=0.0)
    parser.add_argument("-r", help="Angular velocity, in rad/s", type=float, default=0.0)
    parser.add_argument("-t", help="Duration to run at constant velocity after ramping, in seconds", type=float, default=2.0)
    parser.add_argument("--effort", help="Should we plot joint effort?", action="store_true")
    parser.add_argument("--velocity", help="Should we plot joint velocity?", action="store_true")
    args, unknown = parser.parse_known_args()

    if args.x == 0.0 and args.r == 0.0:
        print("Must specify a velocity")
        exit(-1)

    rclpy.init()

    b = BaseTest(args.x, args.r, args.effort, args.velocity)

    thread = threading.Thread(target=rclpy.spin, args=(b,), daemon=True)
    thread.start()

    time.sleep(1)
    b.run(args.t)

    b.pub.publish(Twist())

    b.plot()
