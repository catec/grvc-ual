#!/usr/bin/env python
import os
import rospy
import subprocess

def clean_previous_files(folder_path, model_name):    
    try:
        # Iterate through all files in the folder
        for filename in os.listdir(folder_path):
            # Check if the file ends with the specified suffix
            if filename.endswith(model_name):
                # Remove the file
                file_path = os.path.join(folder_path, filename)
                os.remove(file_path)
                rospy.logdebug(f"Removed: {file_path}")
        rospy.loginfo("Previous Files removed successfully.")
    except Exception as e:
        rospy.logerr(f"An error occurred while removing previous command files: {e}")

def clean_previous_cmake_references(path_to_makefile, model_name):
    try:
        # Read the existing content of the file
        with open(path_to_makefile, 'r') as file:
            lines = file.readlines()
        # Open the file in write mode to truncate and rewrite
        with open(path_to_makefile, 'w') as file:
            # Write lines back to the file, excluding those ending with the specified suffix
            for line in lines:
                if not line.strip().endswith(model_name):
                    file.write(line)
        rospy.loginfo("Previous mentions in Cmake file removed successfully.")
    except Exception as e:
        rospy.logerr(f"An error occurred while removing previous lines in cmake file: {e}")

def clean_previous_env(path_to_airframes, path_to_makefile, model_name):
    rospy.loginfo("Cleanin previous references of the airframe at makefile")
    clean_previous_cmake_references(path_to_makefile, model_name)
    rospy.loginfo("Removing previous command files")
    clean_previous_files(path_to_airframes, model_name)
    rospy.loginfo("Environment successfully cleaned")

def make_env(path_to_px4):
    script_path = "px4_make.bash"
    try:
        # Execute the bash script
        subprocess.run(['bash', script_path])
        rospy.loginfo(f"Bash script '{script_path}' executed successfully.")
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"Error executing bash script '{script_path}': {e}")

def get_file_name(model_name):
    hash_upper_limit = 22000
    hash_lower_limit = 22999
    hashed_file_name = str(hash_within_limits(model_name, hash_lower_limit, hash_upper_limit)) + '_' + model_name
    return hashed_file_name

def hash_within_limits(name, lower_limit, upper_limit):
    # Calculate the hash value of the name
    hash_value = hash(name)
    # Map the hash value to the specified range
    mapped_hash = (hash_value % (upper_limit - lower_limit + 1)) + lower_limit
    return mapped_hash

def write_model_to_makefile(model_file_name, makefile_path):
    try:
        with open(makefile_path, 'r') as makefile:
            makefile_lines = makefile.readlines()
            
        try:
            start_marker_text = "Reserve for custom models"
            start_line_index = find_marker_index(makefile_lines, start_marker_text) + 1
        except ValueError as e:
            rospy.logerr(e)
        
        makefile_lines.insert(start_line_index, '\t' + model_file_name + '\n')
            
        with open(makefile_path, 'w') as makefile:
            makefile.writelines(makefile_lines)
        
        rospy.loginfo(f"Model {model_file_name} written at {makefile_path} successfully")
    except Exception as e:
        rospy.logerr(f"An error ocurred while writting makefile: {e}")

def create_px4model_symlinks(px4_airframes_path, model_cmd_source_path):
    create_symlink(model_cmd_source_path, px4_airframes_path)
    if os.path.exists(model_cmd_source_path + '.post'):
        create_symlink(model_cmd_source_path + '.post', px4_airframes_path)


def create_symlink(source_path, target_path):
    try:
        os.symlink(source_path, target_path)
        rospy.loginfo(f"Symbolic link created: {target_path} -> {source_path}")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")

def find_marker_index(text_lines, marker_text):
    for index, line in enumerate(text_lines):
        if marker_text in line:
            return index
    raise ValueError(f"Marker text '{marker_text}' not found in any line.")