#! /usr/bin/env python

# Copyright (c) 2017, Rethink Robotics Inc.
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

import rospy
import argparse
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_interface import Limb

def main():
    """
    Send a STOP command to the motion controller, which will safely stop the
    robot if it is moving. This is useful when the robot is executing a long
    trajectory that needs to be canceled. It also shows how to make the call
    to the motion_controller_action_client to send the STOP message.

    $ rosrun intera_examples stop_motion_trajectory.py
    """

    try:
        rospy.init_node('stop_motion_trajectory')
        limb = Limb()
        traj = MotionTrajectory(limb = limb)
        result = traj.stop_trajectory()
        if result is None:
            rospy.logerr('FAILED to send stop request')
            return

        if result.result:
            rospy.loginfo('Motion controller successfully stopped the motion!')
        else:
            rospy.logerr('Motion controller failed to stop the motion: %s',
                         result.errorId)
    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. Exiting before stop completion.')


if __name__ == '__main__':
    main()
