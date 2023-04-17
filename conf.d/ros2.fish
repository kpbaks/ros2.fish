not status is-interactive; and return

# variable namespace = ROS2_FISH

function _ros2_install --on-event ros2_install
    # Set universal variables, create bindings, and other initialization logic.
    if not set -q ROS2_FISH_ABBR_OR_ALIAS
        set -Ux ROS2_FISH_ABBR_OR_ALIAS abbr
    end
end

function _ros2_update --on-event ros2_update
    # Migrate resources, print warnings, and other update logic.
end

function _ros2_uninstall --on-event ros2_uninstall
    # Erase "private" functions, variables, bindings, and other uninstall logic.
    set --erase ROS2_FISH_ABBR_OR_ALIAS
    set --erase ROS2_FISH_VERBOSE
end


set -g ROS2_FISH_VERBOSE

function __ros2_fish_echo
    if set -q ROS2_FISH_VERBOSE
        set_color normal
        printf "%s%s%s %s\n" (set_color --reverse) (status current-filename) (set_color normal) (string join " " $argv)
    end
end

# Every ROS2 installation should have a /opt/ros directory
if not test -d /opt/ros/
    __ros2_fish_echo "/opt/ros/ not found"
    return 0
end

if not type -q bass
    __ros2_fish_echo "bass (https://github.com/edc/bass) not installed"
    return 0
end

if not set -q ROS_DISTRO
    # ROS_DISTRO
    pushd /opt/ros

    # distribution taken from https://docs.ros.org/en/galactic/Releases.html#list-of-distributions
    # checked in order of release date
    # distributions that have reached EOL are not included (as of 2023-03-06)
    if test -d ./humble
        set -gx ROS_DISTRO humble
    else if test -d ./galactic
        set -gx ROS_DISTRO galactic
    else if test -d ./foxy
        set -gx ROS_DISTRO foxy
    end

    popd
end

bass source /opt/ros/$ROS_DISTRO/setup.bash

set -l argcomplete
if command -q register-python-argcomplete
    set argcomplete register-python-argcomplete
else if command -q register-python-argcomplete3
    set argcomplete register-python-argcomplete3
else
    __ros2_fish_echo "register-python-argcomplete not installed"
    return 0
end

# TODO: register-python-argcomplete does not work with fish
# for cmd in ros2 colcon rosdep
#     $argcomplete --shell fish $cmd | source
# end

# ROS2 aliases/abbreviations

alias rtl 'ros2 topic list'
alias rnl 'ros2 node list'
alias rsl 'ros2 service list'
alias rpl 'ros2 pkg list'

if command --query fzf
    if not set -q ROS2_FISH_FZF_OPTS
        set -g ROS2_FISH_FZF_OPTS --reverse \
            --height 40% \
            --border
    end

    function rtt
        ros2 topic list \
            | fzf $ROS2_FISH_FZF_OPTS --preview 'ros2 topic type {}'
    end

    function rti
        ros2 topic list \
            | fzf $ROS2_FISH_FZF_OPTS --preview 'ros2 topic info {}'
    end
end

abbr -a r ros2
abbr -a rr ros2 run
abbr -a cb colcon build --symlink-install --packages-select
abbr -a rbi --set-cursor ros2 bag info '*%.bag'
abbr -a rbp --set-cursor ros2 bag play '*%.bag'
abbr -a rbr ros2 bag record


# rosdep
abbr -a rd rosdep
abbr -a rdi rosdep init
abbr -a rdu rosdep update
abbr -a rdc rosdep check


# Check if rosdep is installed
if not command --query rosdep
    __ros2_fish_echo "rosdep not installed"
end


# FIX: this runs on false positives
# # Check if rosdep has been initialized
# # `rosdep` looks for a files in `/etc/ros/rosdep/sources.list.d/`
# # If the directory does not exist, or there are no files in it, then rosdep has not been initialized
# if not test -d /etc/ros/rosdep/sources.list.d/ -o (count (command ls /etc/ros/rosdep/sources.list.d/)) -gt 0
#     # Initialize rosdep
#     __ros2_fish_echo "rosdep not initialized"
#     __ros2_fish_echo "initializing rosdep"
#     # Check if user is root. The root user has id 0
#     if test (id -u) -eq 0
#         __ros2_fish_echo "running rosdep init as root"
#         rosdep init
#     else
#         __ros2_fish_echo "running rosdep init as user"
#         sudo rosdep init
#     end
# end
#
# # Check if rosdep has been updated
# # After rosdep has been initialized, it needs to be updated
# # Calling `rosdep update` will update the rosdep database, and write a cache 
# # to ~/.ros/rosdep/{meta,source}_cache
# # if the cache does not exist, then rosdep has not been updated
# if not test -f ~/.ros/rosdep/meta_cache.yaml -o -f ~/.ros/rosdep/source_cache.yaml
#     # Update rosdep
#     __ros2_fish_echo "rosdep not updated"
#     __ros2_fish_echo "updating rosdep"
#     rosdep update
#     __ros2_fish_echo "rosdep updated"
# end
