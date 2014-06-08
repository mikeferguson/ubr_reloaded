/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Derived a bit from teleop_pr2_keyboard
 *  Copyright (c) 2013-2014, Unbounded Robotics Inc.
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Unbounded Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Michael Ferguson, Kevin Watts */

#include <ros/ros.h>
#include <ubr_teleop/robot_controller.h>

#include <termios.h>
#include <signal.h>

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT  0x44
#define KEYCODE_UP    0x41
#define KEYCODE_DOWN  0x42

int kfd = 0;
struct termios cooked, raw;
RobotController robot;

void quit(int sig)
{
  robot.stop();
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

void callback(const ros::TimerEvent&)
{
  robot.sendCommands();
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "teleop");

  ros::NodeHandle n("~");
  robot.init(n);
  robot.start();

  ros::Timer timer = n.createTimer(ros::Duration(0.05), callback);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  signal(SIGINT, quit);

  char c;
  bool dirty=false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'WASD' to translate");
  puts("Use arrow keys to move head");
  puts("Any other key to stop");

  while (ros::ok())
  {
    // get the next event from the keyboard
    if (read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    switch (c)
    {

    case KEYCODE_W:
      robot.setBaseVelocity(0.5, 0);
      break;
    case KEYCODE_S:
      robot.setBaseVelocity(-0.5, 0);
      break;
    case KEYCODE_A:
      robot.setBaseVelocity(0, 1.0);
      break;
    case KEYCODE_D:
      robot.setBaseVelocity(0, -1.0);
      break;

    case KEYCODE_RIGHT:
      robot.setHeadPosition(robot.getHeadPan() - 0.1, robot.getHeadTilt());
      break;
    case KEYCODE_LEFT:
      robot.setHeadPosition(robot.getHeadPan() + 0.1, robot.getHeadTilt());
      break;

    case KEYCODE_DOWN:
      robot.setHeadPosition(robot.getHeadPan(), robot.getHeadTilt() - 0.1);
      break;
    case KEYCODE_UP:
      robot.setHeadPosition(robot.getHeadPan(), robot.getHeadTilt() + 0.1);
      break;

    default:
      robot.stop();
    }
  }

  return 0;
}
