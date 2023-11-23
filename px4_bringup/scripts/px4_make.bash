#!/bin/bash
# Get the project folder path from the environment variable PX4_PATH
project_folder="$PX4_PATH"

# Check if the project folder is set
if [ -z "$project_folder" ]; then
    echo "Error: Environment variable PX4_PATH is not set."
    exit 1
fi

# Check if the project folder exists
if [ ! -d "$project_folder" ]; then
    echo "Error: Project folder '$project_folder' not found."
    exit 1
fi

# Change to the project folder
cd "$project_folder" || exit 1

# Run the make command
make

# Check the exit status of the make command
if [ $? -ne 0 ]; then
    echo "Error: Make command failed."
    exit 1
fi

echo "Project successfully built."
exit 0
