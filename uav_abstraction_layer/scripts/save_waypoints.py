#!/usr/bin/env python
import argparse
import subprocess
import sys
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped

current_pose = PoseStamped()
def pose_callback(data):
    global current_pose
    current_pose = data

def save_waypoints():
    # Parse arguments
    parser = argparse.ArgumentParser(description='Save waypoints to a yaml file')
    parser.add_argument('-plan_package', type=str, default='uav_abstraction_layer',
                        help='Name of the package where generated plan will be stored')
    parser.add_argument('-plan_folder', type=str, default='',
                        help='Name of the folder inside plan_package where generated plan will be stored')
    parser.add_argument('-plan_file_prefix', type=str, default='recorded_wp',
                        help='Prefix for the generated plan file')
    
    args, unknown = parser.parse_known_args()
    # utils.check_unknown_args(unknown)

    rospy.init_node('waypoint_saver')

    # Subscribe to pose topic
    pose_url = 'ual/pose'
    rospy.Subscriber(pose_url, PoseStamped, pose_callback)

    # Wait for pose topic
    rospy.loginfo('Waiting for pose topic...')
    rospy.wait_for_message(pose_url, PoseStamped)
    rospy.loginfo('Pose topic found!')


    # Get plans directory
    if args.plan_folder == '':
        plans_dir = rospkg.RosPack().get_path(args.plan_package) + '/plans/'
    else:
        if not args.plan_folder.endswith('/'):
            args.plan_folder = args.plan_folder + '/'

        if not args.plan_folder.startswith('/'):
            plans_dir = rospkg.RosPack().get_path(args.plan_package) + '/' + args.plan_folder 
        else:
            plans_dir = args.plan_folder
    subprocess.call("mkdir -p " + plans_dir, shell=True)
        
    # Get file name with timestamp
    file_name = args.plan_file_prefix + '_' + rospy.get_time().__str__().split('.')[0] + '.yaml'
    
    # Open yaml file and save frame_id
    frame_id = current_pose.header.frame_id
    rospy.loginfo('All waypoints in frame_id: %s', frame_id)
    file_url = plans_dir + file_name
    yaml_file = open(file_url, 'w')
    yaml_file.write('frame_id: ' + frame_id + '\n')
    rospy.loginfo('Waypoints will be saved to: %s', file_url)

    # Check we are actually receiving pose
    rospy.loginfo('Reading pose from: %s', pose_url)
    if current_pose.header.frame_id == '':
        rospy.logerr('Unable to read pose, assure it is being published!')
        return
    
    for wp_id in range(1000):
        if input("Press Enter to save current pose or q to quit and save curent ... ") == 'q':
            break        

        # print(current_pose) # debug!
        wp_to_save = 'wp_' + str(wp_id) + ': [' + \
            str(current_pose.pose.position.x) + ', ' + \
            str(current_pose.pose.position.y) + ', ' + \
            str(current_pose.pose.position.z) + ', ' + \
            str(current_pose.pose.orientation.x) + ', ' + \
            str(current_pose.pose.orientation.y) + ', ' + \
            str(current_pose.pose.orientation.z) + ', ' + \
            str(current_pose.pose.orientation.w) + ']'

        # Check waypoint frame_id
        current_frame_id = current_pose.header.frame_id
        if current_frame_id != frame_id:
            rospy.logerr("Waypoint NOT SAVED: Current frame_id [%s] differs from expected [%s]", current_frame_id, frame_id)
            continue
        rospy.loginfo('Saving current pose as %s', wp_to_save)
        yaml_file.write(wp_to_save + '\n')

    if wp_id > 0:
        rospy.loginfo('All waypoints saved to: %s', file_url)
    yaml_file.close()
    return

if __name__ == "__main__":
    save_waypoints()
