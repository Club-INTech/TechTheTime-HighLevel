#! /bin/bash

#README :
# To use this command, you need to be in the parent of the src file of your package, and enter the package name as a parameter
# Do not forget to add the user rights
#
# Alias : alias <command name>="bash <path/to/this>"



if ! [ $# -eq 1 ]; then
    echo "a package name is required"
    exit 1
fi

colcon build --packages-select $1

source ./install/setup.bash

echo "done"
exit 0