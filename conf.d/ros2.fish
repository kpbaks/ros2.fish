not status is-interactive; and return

# variable namespace = ROS2_FISH

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

if not command --query register-python-argcomplete
    set -q ROS2_FISH_VERBOSE
    and printf "%s %s\n" (status current-filename) "register-python-argcomplete not installed"
    return
end

for cmd in ros2 colcon
    register-python-argcomplete --shell fish $cmd | source
end

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
