#!/bin/bash

# Make sure we should the system drivers.
export LIBGL_DRIVERS_PATH="/usr/lib/x86_64-linux-gnu/dri"

RUN_MANGOHUD=false
if [ -n $1 ] && [ "$1" = "--run-mangohud" ]
then
    RUN_MANGOHUD=true
    export MANGOHUD_DLSYM=1
    mangohud ./build/bin/bpt
else
    ./build/bin/bpt
fi
