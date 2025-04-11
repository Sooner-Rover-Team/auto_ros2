#!/usr/bin/bash

# basically an OnError Goto stop. prevents some nonsense
set -e

# if the system supports SELinux, this command will almost always be available.
#
# so, we'll start off by checking if we've got SELinux support using this test,
# which only checks if the `selinuxenabled` command is present.
if [ command -v selinuxenabled &> /dev/null ]; then

    # now, we'll run the command to see if SELinux is turned on.
    SELINUX_IS_ON=$(selinuxenabled)

    # if this is `0`, it's on!
    #
    # that means we'll need to check if our repo is protected.
    if [ "$depth" -eq "0" ]; then

        # check if the repo is protected...
        if [ ! /usr/bin/ls -Zd "$AUTO_ROS2_REPO_LOCATION" | grep -q "container_file_t" ]; then
            # it isn't. let's prompt the user to fix it...
            echo "The \`auto_ros2\` repo is protected by SELinux. To mount it, we'll need to add an exception..."
            echo "Please enter your password if prompted."
            sudo chcon --recursive --type svirt_sandbox_file_t "$AUTO_ROS2_REPO_LOCATION"
            echo "Protection exception added successfully!"
        fi
    fi
fi
