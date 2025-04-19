#!/usr/bin/bash

# stop script on error
set -e

if [ "$(uname)" = "Linux" ]; then
    # don't use `sudo` for podman...
    if command -v podman &> /dev/null; then
        declare -x OPTIONAL_SUDO=""
        echo "\`sudo\` is not expected to be required for Docker on this system."
    else
        # note: the `-E` flag in `sudo -E` is veeery important!
        #
        # it preserves the environment when running with `sudo`, meaning that the
        # variable we created above is maintained!
        #
        # please ensure this is never removed.
        declare -x OPTIONAL_SUDO="sudo -E"
        echo "\`sudo\` seems necessary for Docker on this system."
    fi
else
    declare -x OPTIONAL_SUDO=""
    echo "\`sudo\` is not expected to be required for Docker on this system."
fi
