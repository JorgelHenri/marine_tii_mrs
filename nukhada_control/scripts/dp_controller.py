#!/usr/bin/env python
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
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

# Import the ROS Python library
import rospy
# Import the NumPy package for numerical computations
import numpy as np
# Import the dynamic positioning controller base class to inherit methods such as error computation update,
# publishing from some important ROS topics (e.g. trajectory, pose and velocity reference) and access to
# the vehicle model class, that is necessary to explicitly use the vehicle model
from uuv_control_interfaces import DPControllerBase
from PID import PIDRegulator
import tf
import tf.transformations as trans

from dynamic_reconfigure.server import Server
from nukhada_control.cfg import PIDConfig

class DPController(DPControllerBase):
    # A new controller that is based on the DPControllerBase must at least provide the implementation of
    # the method update_controller.
    # The _reset_controller method can also be overridden and it will be called every time there is a service call
    # <vehicle namespace>/reset_controller. The default implementation sets the reference and error vectors to
    # zero.
    # The update_controller method must contain the implementation of the control algorithm and will be called
    # by every update of the vehicle's odometry message. It is therefore not necessary to explicitly call this update
    # function in this controller implementation.
    # For the controller to send the control torques to the vehicle's thruster manager, at the end of the
    # update_controller function the 6 x 1 control vector (type numpy.ndarray) sent using the function
    # publish_control_wrench from the super class, which will generate a Wrench ROS message and publish it to the
    # correspondent thruster manager node.
    # For this tutorial, a simple PID controller will be implemented. The controller's control torque output is
    # "tau" therefore computed as:
    #
    #   tau = Kp * e + Kd * de/dt + Ki int_e
    #
    # where e is the pose error vector, in this case defined as e = (x, y, z, roll, pitch, yaw)^T

    def __init__(self, params_srv):
        # Calling the constructor of the super-class DPControllerBase, which has the implementation of the error
        # computation update and to publish the resulting torque control vector.
        super(DPController, self).__init__(self)

        # Let's initialize the controller gain matrices Kp, Kd and Ki
        self._Kp = np.zeros(3)
        self._Kd = np.zeros(3)
        self._Ki = np.zeros(3)
        # Initialize the integrator component
        self._int = np.zeros(shape=(6,))
        # Initialize variable that will store the vehicle pose error
        self._error_pose = np.zeros(shape=(6,))
        self._params_srv = params_srv
        self._init_pid_controllers()

        self._is_init = True

    def _reset_controller(self):
        # The _reset_controller method from the super class DPControllerBase already sets the error
        # and reference vectors to zero, but this class has additional attributes that should also
        # be taken care of.
        # This implementation will, therefore, first call the super class reset method
        super(DPController, self)._reset_controller()
        # And then proceed to set the internal variables back to zero
        self._error_pose = np.zeros(shape=(6,))
        self._int = np.zeros(shape=(6,))

    def update_controller(self):
        if not self._is_init:
            return False
        # The controller algorithm must be implemented here, the super class will connect this method
        # to the odometry update as a callback function

        # First test whether or not the odometry topic subscriber has already been initialized
        if not self.odom_is_init:
            return

        self._update_pid_controllers()
        
        t = rospy.get_time()

        vx  = self.pid_forward.regulate(self._error_pose[0], t)
        wx  = self.pid_lateral.regulate(np.arctan2(self._error_pose[1],self._error_pose[0]), t)
        wx += self.pid_heading.regulate(self._error_pose[5], t)

        self._error_pose = self.error_pose_euler

        v_linear = np.array([vx, 0, 0])
        v_angular = np.array([0, 0, wx])

        # Compute the control forces and torques using the current error vectors available
        tau = np.concatenate((v_linear, v_angular), axis=None)
        
        # Use the super class method to convert the control force vector into a ROS message
        # and publish it as an input to the vehicle's thruster manager. The thruster manager module
        # will then distribute the efforts amongst the thrusters using the thruster allocation matrix
        self.publish_control_wrench(tau)


    def _init_pid_controllers(self):
        self._update_pid_coefficients()

        self.pid_forward = PIDRegulator(self._Kp[0], self._Ki[0], self._Kd[0], 20)
        self.pid_lateral = PIDRegulator(self._Kp[1], self._Ki[1], self._Kd[1], 20)
        self.pid_heading = PIDRegulator(self._Kp[2], self._Ki[2], self._Kd[2], 20)


    def _update_pid_controllers(self):
        self._update_pid_coefficients()

        # This is very bad
        self.pid_forward.p = self._Kp[0]
        self.pid_forward.i = self._Ki[0]
        self.pid_forward.d = self._Kd[0]

        self.pid_lateral.p = self._Kp[1]
        self.pid_lateral.i = self._Ki[1]
        self.pid_lateral.d = self._Kd[1]

        self.pid_heading.p = self._Kp[2]
        self.pid_heading.i = self._Ki[2]
        self.pid_heading.d = self._Kd[2]


    def _update_pid_coefficients(self):
        self._Kp = [self._params_srv.config.Kp_f, self._params_srv.config.Kp_l, self._params_srv.config.Kp_h]
        self._Ki = [self._params_srv.config.Ki_f, self._params_srv.config.Ki_l, self._params_srv.config.Ki_h]
        self._Kd = [self._params_srv.config.Kd_f, self._params_srv.config.Kd_l, self._params_srv.config.Kd_h]

def params_callback(config, level):
    return config


if __name__ == '__main__':
    print('Nukhada - DP Controller')
    rospy.init_node('dp_controller')

    try:
        params_srv = Server(PIDConfig, params_callback)
        node = DPController(params_srv)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
