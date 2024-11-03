#!/usr/bin/env python3

# Copyright (C) 2024 Michael Ferguson
# Copyright (C) 2015 Fetch Robotics Inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: Michael Ferguson

import sys
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType


class CameraReconfigure(Node):

    def __init__(self):
        super().__init__('camera_reconfigure')
        self.client = self.create_client(SetParameters,
                                         '/head_camera/driver/set_parameters')
        while not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Waiting for service')

    def enable_auto(self, enable):
        exposure = Parameter()
        exposure.name = 'auto_exposure'
        exposure.value.type = ParameterType.PARAMETER_BOOL
        exposure.value.bool_value = enable

        white_balance = Parameter()
        white_balance.name = 'auto_white_balance'
        white_balance.value.type = ParameterType.PARAMETER_BOOL
        white_balance.value.bool_value = enable

        request = SetParameters.Request()
        request.parameters = [exposure, white_balance]

        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        result = self.future.result()
        for result in result.results:
            if not result.successful:
                self.get_logger().warn('Unable to set parameter')
                return
        self.get_logger().info('Camera configured')


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: camera_reconfigure --enable/disable")
        exit(-1)

    rclpy.init()
    reconfigure = CameraReconfigure()

    if sys.argv[1] == "--enable":
        reconfigure.enable_auto(True)
    else:
        reconfigure.enable_auto(False)
