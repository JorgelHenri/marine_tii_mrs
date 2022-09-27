#!/usr/bin/env python

import rospy as rp
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from os.path import isdir, join
import ruamel.yaml
import sys

if __name__ == '__main__':
    rp.init_node('wp_from_coverage_planner', anonymous=True)

    if rp.is_shutdown():
        rp.logerr('ROS master not running!')
        sys.exit(-1)

    if rp.has_param('~filename'):
        filename = rp.get_param('~filename')
    else:
        raise rp.ROSException('No output file given')

    if rp.has_param('~output_dir'):
        output_dir = rp.get_param('~output_dir')
        if not isdir(output_dir):
            raise rp.ROSException('Invalid output directory, output_dir=' + output_dir)
    else:
        raise rp.ROSException('No output dir given')

    if rp.has_param('~max_forward_speed'):
        max_forward_speed = rp.get_param('~max_forward_speed')
    else:
        raise rp.ROSException('No output dir given')

    try:
        pose_array = rp.wait_for_message('/waypoint_list', PoseArray, timeout=5)
    except rp.ROSException:
        raise rp.ROSException('Message not received! Closing node...')

    wp_yaml = {'inertial_frame_id': 'world', 'waypoints':[]}
    
    for p in pose_array.poses:    
        wp_yaml['waypoints'].append({
            'point': [p.position.x, p.position.y, 0],
            'max_forward_speed': max_forward_speed,
            'heading': 0,
            'use_fixed_heading': False
        })

    try:
        with open(join(output_dir, filename), 'w') as file:
            ruamel.yaml.dump(wp_yaml, file)

        rp.loginfo('Waypoints file successfully created, filename=%s', filename)
    except Exception as e:
        rp.logerr(('Failed to convert waypoints: ', str(e)))