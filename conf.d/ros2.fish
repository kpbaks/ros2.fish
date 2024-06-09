status is-interactive; or return 0

begin
    echo $version | read --delimiter . major minor patch
    if not test $major -ge 3 -a $minor -ge 6
        set -l reset (set_color normal)
        printf "[%sros2.fish%s] %serror%s: minimum required %sfish%s version is %s3.6.0%s, but you have %s%s%s\n" (set_color blue) $reset (set_color red) $reset (set_color $fish_color_command) $reset (set_color green) $reset (set_color red) $version $reset
        return 0
    end
end

function _ros2_install --on-event ros2_install
    # set_color green
    # echo "ros2.fish install"
    # set_color normal
    # Set universal variables, create bindings, and other initialization logic.
    if not set -q ROS2_FISH_ABBR_OR_ALIAS
        set -Ux ROS2_FISH_ABBR_OR_ALIAS abbr
    end
end

function _ros2_update --on-event ros2_update
    # set_color yellow
    # echo "ros2.fish update"
    # set_color normal
    # Migrate resources, print warnings, and other update logic.
end

function _ros2_uninstall --on-event ros2_uninstall
    # set_color red
    # echo "ros2.fish uninstall"
    # set_color normal

    # Erase "private" functions, variables, bindings, and other uninstall logic.
    set --erase ROS2_FISH_ABBR_OR_ALIAS
    set --erase ROS2_FISH_VERBOSE
end


set -g ROS2_FISH_VERBOSE

# TODO: make a logging system similar to tldr-on-error.fish
function __ros2_fish_echo
    if set -q ROS2_FISH_VERBOSE
        set_color normal
        printf "%s%s%s %s\n" (set_color --reverse) (status current-filename) (set_color normal) (string join " " $argv)
    end
end

set -g ROS2_FISH_ABBRS
function __ros2_fish_abbr_add --argument-names name
    set -l abbr_args $argv[2..-1]
    set --append ROS2_FISH_ABBRS $name
    abbr --add $name $abbr_args
end

function __ros2_fish_abbr_list
    for abbr in $ROS2_FISH_ABBRS
        echo $abbr
    end
end

set -g ROS2_FISH_ALIASES
function __ros2_fish_alias_add --argument-names name
    set -l alias_args $argv[2..-1]
    set --append ROS2_FISH_ALIASES $name
    alias $name "$alias_args"
end

function __ros2_fish_alias_list
    for alias in $ROS2_FISH_ALIASES
        echo $alias
    end
end

function ros2.fish
    # TODO: make prettier
    set -l subcommand $argv[1]
    switch $subcommand
        case list
            __ros2_fish_abbr_list
            __ros2_fish_alias_list
        case '*'
            echo "ros2-fish: unknown subcommand $subcommand"
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
    else if test -d ./jazzy
        set -gx ROS_DISTRO jazzy
    end

    popd
end

__ros2_fish_echo "sourcing /opt/ros/$ROS_DISTRO/setup.bash"
bass source /opt/ros/$ROS_DISTRO/setup.bash

set -l argcomplete
if command -q register-python-argcomplete
    set argcomplete register-python-argcomplete
else if command -q register-python-argcomplete3
    set argcomplete register-python-argcomplete3
else
    __ros2_fish_echo "register-python-argcomplete not installed"
    # return 0
end

# TODO: register-python-argcomplete does not work with fish
# for cmd in ros2 colcon rosdep
#     $argcomplete --shell fish $cmd | source
# end

# status --fil

# ROS2 aliases/abbreviations



__ros2_fish_alias_add rtl 'ros2 topic list --show-types | ~/.config/fish/functions/__as-tree.py --color'
__ros2_fish_alias_add rnl 'ros2 node list | ~/.config/fish/functions/__as-tree.py --color'
__ros2_fish_alias_add rsl 'ros2 service list'
__ros2_fish_alias_add rpl 'ros2 pkg list'

if command --query fzf
    if not set -q ROS2_FISH_FZF_OPTS
        set -g ROS2_FISH_FZF_OPTS --reverse \
            --height 50% \
            --border
    end

    __ros2_fish_alias_add rtt "ros2 topic list | fzf $ROS2_FISH_FZF_OPTS --preview 'ros2 topic type {}'"
    # function rtt
    #     ros2 topic list \
    #         | fzf $ROS2_FISH_FZF_OPTS --preview 'ros2 topic type {}'
    # end

    __ros2_fish_alias_add rti "ros2 topic list | fzf $ROS2_FISH_FZF_OPTS --preview 'ros2 topic info {}'"
    # function rti
    #     ros2 topic list \
    #         | fzf $ROS2_FISH_FZF_OPTS --preview 'ros2 topic info {}'
    # end

else
    __ros2_fish_echo "fzf (https://github.com/junegunn/fzf) not installed. Aliases rtt and rti not created"
end

__ros2_fish_abbr_add r ros2
__ros2_fish_abbr_add rr ros2 run
__ros2_fish_abbr_add rl ros2 launch
__ros2_fish_abbr_add cb colcon build --symlink-install --packages-select


function abbr_ros2_bag_play
    # search all directories in the current directory for a `metadata.yaml` file
    # if there is only one, then append the directory it is in to the command
    set -l metadata_files (command find . -maxdepth 1 -type f -name metadata.yaml)
    set -l cmd ros2 bag play
    if test (count $metadata_files) -eq 1
        # if builtin --query path
        set --append cmd (path dirname $metadata_files[1])
        # else
        #     set --append cmd (dirname $metadata_files[1])
        # end
    end
    echo -- $cmd
end

function abbr_ros2_bag_info
    # search all directories in the current directory for a `metadata.yaml` file
    # if there is only one, then append the directory it is in to the command
    set -l metadata_files (find . -maxdepth 1 -type f -name metadata.yaml)
    set -l cmd ros2 bag info
    if test (count $metadata_files) -eq 1
        # if builtin --query path
        set --append cmd (path dirname $metadata_files[1])
        # else
        #     set --append cmd (dirname $metadata_files[1])
        # end
    end
    echo -- $cmd
end

__ros2_fish_abbr_add rbi --set-cursor --function abbr_ros2_bag_info
__ros2_fish_abbr_add rbp --set-cursor --function abbr_ros2_bag_play
__ros2_fish_abbr_add rbr ros2 bag record

# rosdep
__ros2_fish_abbr_add rd rosdep
__ros2_fish_abbr_add rdi rosdep init
__ros2_fish_abbr_add rdu rosdep update
__ros2_fish_abbr_add rdc rosdep check


# Check if rosdep is installed
if not command --query rosdep
    __ros2_fish_echo "rosdep not installed."
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
