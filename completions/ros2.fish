set -l C complete --command ros2

set -g __fish_ros2 /opt/ros/$ROS_DISTRO/bin/ros2
set -g ros2_exe (command --search ros2)


# Check if the command line's second and third arguments are the same as the given subcommand and subsubcommand
function __fish_seen_subcommand_with_subsubcommand --argument-names subcommand subsubcommand
    set -l cmd (commandline -poc)
    set -e cmd[1]

    if test (count $cmd) -lt 2
        return 1
    end
    if test $cmd[1] != $subcommand
        return 1
    end
    if test $cmd[2] != $subsubcommand
        return 1
    end
    return 0
end

# Check if command line's second argument is the same as the given subcommand and has more than 2 arguments
function __fish_seen_subcommand_with_argument --argument-names subcommand
    set -l cmd (commandline -poc)
    set -e cmd[1]
    if test (count $cmd) -lt 2
        return 1
    end
    if test $cmd[1] != $subcommand
        return 1
    end
    return 0
end

# Check if anything is being called with the nth argument/token is contained in the given array. Tis can be helpful, e.g.:
# - with n=-1 to check whether `ros2 topic echo` is being called with a topic name already given
# - with n=-2 to check whether a topic name has been given plus with n=-1 to check whether a message type has been given
function __fish_seen_nth_arg_from --argument-names n
    set -l cmd (commandline -poc)

    if contains -- $cmd[$n] $argv[2..]
        return 0
    end

    return 1
end


# Print all files recursively in the current directory with the given extension
function __fish_print_files_in_subdirectories_with_extension --argument-names extension
    command find . -type f -name "*.$extension" -printf '%P\n'
end

# Count the number of leading spaces in a string
function count_leading_spaces --argument-names str
    set -l leading_spaces (string match -r '^ *' -- $str)
    echo (string length -- $leading_spaces)
end

# Convert the output of the `ros2 interface proto` command to a YAML string to be used in ros2 topic pub or ros2 service call
function proto_output_to_yaml
    printf "{"
    for i in (seq (count $argv))
        set -l current_line $argv[$i]
        set -l current_line_trimmed (string trim $current_line)

        if not test -n $current_line_trimmed
            continue
        end

        printf "%s" $current_line_trimmed

        set -l last_char (string sub --start -1 -- $current_line)
        set -l next_i (math $i + 1)
        set -l next_line $argv[$next_i]
        if test $last_char = ":"
            printf " {"
        else if set -q argv[$next_i] # There is a next line
            if test (count_leading_spaces $current_line) -gt (count_leading_spaces $next_line)
                printf "}"
            end

            if set -q argv[(math $next_i + 1)]
                # Last line is empty, so if there's no next next line, we don't print a comma
                printf ", "
            end
        end
    end
    printf "}"
end

set -g __fish_ros2_all_commands action bag component daemon doctor interface launch lifecycle multicast node param pkg run security service topic wtf
set -g __fish_ros2_action_subcommands info list send_goal
set -g __fish_ros2_bag_subcommands info play record record-pause resume rewind
set -g __fish_ros2_component_subcommands list load unload
set -g __fish_ros2_daemon_subcommands start stop
set -g __fish_ros2_doctor_subcommands check
set -g __fish_ros2_interface_subcommands list package show type
set -g __fish_ros2_launch_subcommands list
set -g __fish_ros2_lifecycle_subcommands get get-available-states get-available-transitions get-state list list-nodes set-state
set -g __fish_ros2_multicast_subcommands disable enable
set -g __fish_ros2_node_subcommands info list
set -g __fish_ros2_param_subcommands describe get get-names list set
set -g __fish_ros2_pkg_subcommands create list prefix search
set -g __fish_ros2_run_subcommands
set -g __fish_ros2_security_subcommands create_key create_keystore create_permission create_token generate_artifacts generate_policy
set -g __fish_ros2_service_subcommands call list type
set -g __fish_ros2_topic_subcommands echo hz info list pub type
set -g __fish_ros2_wtf_subcommands

set -g __fish_ros2_all_subcommands \
    $__fish_ros2_action_subcommands \
    $__fish_ros2_bag_subcommands \
    $__fish_ros2_component_subcommands \
    $__fish_ros2_daemon_subcommands \
    $__fish_ros2_doctor_subcommands \
    $__fish_ros2_interface_subcommands \
    $__fish_ros2_launch_subcommands \
    $__fish_ros2_lifecycle_subcommands \
    $__fish_ros2_multicast_subcommands \
    $__fish_ros2_node_subcommands \
    $__fish_ros2_param_subcommands \
    $__fish_ros2_pkg_subcommands \
    $__fish_ros2_run_subcommands \
    $__fish_ros2_security_subcommands \
    $__fish_ros2_service_subcommands \
    $__fish_ros2_topic_subcommands \
    $__fish_ros2_wtf_subcommands

# Check if any of the command line tokens is contained in the given array
function __fish_ros2_cmd_in_array
    for i in (commandline -pco)
        # -- is used to provide no options for contains
        # (if $i is equal to --optname without -- will be error)
        if contains -- $i $argv
            return 0
        end
    end

    return 1
end

function __fish_ros2_print_packages
    $__fish_ros2 pkg list
end

function __fish_ros2_print_nodes
    $__fish_ros2 node list
end


function __fish_ros2_print_executables_in_package --argument-names pkg
    $__fish_ros2 pkg executables $pkg \
        | while read pkg exe
        printf '%s\t%s\n' $exe $pkg
    end
end

# Print launch files in a package
function __fish_ros2_print_launch_files_in_package --argument-names pkg
    set -l pkg_path_prefix ($__fish_ros2 pkg prefix $pkg)
    set -l pkg_share_dir $pkg_path_prefix/share/$pkg/launch
    if test -d $pkg_share_dir
        for file in $pkg_share_dir/*.{py,xml,yaml,yml}
            echo $file
        end
    end
end

# Print filenames of all launch files in a package and the package name
function __fish_ros2_print_launch_filenames_in_package_with_desc --argument-names pkg
    set -l launch_files (__fish_ros2_print_launch_files_in_package $pkg)
    for file in $launch_files
            printf '%s\t%s\n' (path basename $file) $pkg
    end
end

# Print filenames of all launch files in a package
function __fish_ros2_print_launch_filenames_in_package --argument-names pkg
    set -l launch_files (__fish_ros2_print_launch_files_in_package $pkg)
    for file in $launch_files
        echo (path basename $file)
    end
end

function __fish_ros2_print_launch_file_args --argument-names pkg launch_file
    set -l argument_desciption ($__fish_ros2 launch --show-args $pkg $launch_file)
    for i in (seq (count $argument_desciption))
        if test (count_leading_spaces $argument_desciption[$i]) -eq 4
            set -l argument_name (string sub --start 6 --end -2 -- $argument_desciption[$i])
            set -l next_i (math $i + 1)
            set -l argument_description (string trim -- $argument_desciption[$next_i])
            printf '%s:=\t%s\n' (string trim -- $argument_name) $argument_description
        end
    end
end

function __fish_ros2_print_msgs
    $__fish_ros2 interface list --only-msgs \
        | tail -n +2 \
        | while read --delimiter / package ignore msg
        printf '%s\t%s\n' $msg $package
    end
end

function __fish_ros2_print_srvs
    $__fish_ros2 interface list --only-srvs \
        | tail -n +2 \
        | while read --delimiter / package ignore srv
        printf '%s\t%s\n' $srv $package
    end
end

function __fish_ros2_print_actions
    $__fish_ros2 interface list --only-actions \
        | tail -n +2 \
        | while read --delimiter / package ignore action
        printf '%s\t%s\n' $action $package
    end
end

function __fish_ros2_print_services
    $__fish_ros2 service list --show-types \
        | while read service type
        printf '%s\t%s\n' $service (string sub --start 2 --end -1 $type) # remove "[" and "]"
    end
end


function __fish_ros2_print_topics
    $__fish_ros2 topic list --show-types \
        | while read topic type
        printf '%s\t%s\n' $topic (string sub --start 2 --end -1 $type) # remove "[" and "]"
    end
end

function __fish_ros2_print_available_lifecycle_transisions --argument-names node
    set -l lines ($__fish_ros2 lifecycle list $node 2>/dev/null)
    for line in $lines

        set -l first_char (string sub --length 1 -- $line)
        if test $first_char != -
            continue
        end

        set -l tokens (string split " " -- $line)
        printf '%s\t%s\n' $tokens[2] (string sub --start 2 --end -1 $tokens[3])
    end
end

# usage: ros2 [-h] Call `ros2 <command> -h` for more detailed usage. ...
#
# ros2 is an extensible command-line tool for ROS 2.
#
# optional arguments:
#   -h, --help            show this help message and exit
#
# Commands:
#   action     Various action related sub-commands
#   bag        Various rosbag related sub-commands
#   component  Various component related sub-commands
#   daemon     Various daemon related sub-commands
#   doctor     Check ROS setup and other potential issues
#   interface  Show information about ROS interfaces
#   launch     Run a launch file
#   lifecycle  Various lifecycle related sub-commands
#   multicast  Various multicast related sub-commands
#   node       Various node related sub-commands
#   param      Various param related sub-commands
#   pkg        Various package related sub-commands
#   run        Run a package specific executable
#   security   Various security related sub-commands
#   service    Various service related sub-commands
#   topic      Various topic related sub-commands
#   wtf        Use `wtf` as alias to `doctor`
#
#   Call `ros2 <command> -h` for more detailed usage.


$C -f # disable file completion

$C -s h -l help -d "show this help message and exit"

set -l ros2_commands action bag component daemon doctor interface launch lifecycle multicast node param pkg run security service topic wtf
set -l ros2_command_extensions control

# If we haven't typed any of the first-level commands, suggest them
$C -n "not __fish_seen_subcommand_from $ros2_commands $ros2_command_extensions" -a action -d "action related sub-commands"
$C -n "not __fish_seen_subcommand_from $ros2_commands $ros2_command_extensions" -a bag -d "rosbag related sub-commands"
$C -n "not __fish_seen_subcommand_from $ros2_commands $ros2_command_extensions" -a component -d "component related sub-commands"
$C -n "not __fish_seen_subcommand_from $ros2_commands $ros2_command_extensions" -a daemon -d "daemon related sub-commands"
$C -n "not __fish_seen_subcommand_from $ros2_commands $ros2_command_extensions" -a doctor -d "Check ROS setup and other potential issues"
$C -n "not __fish_seen_subcommand_from $ros2_commands $ros2_command_extensions" -a interface -d "Show information about ROS interfaces"
$C -n "not __fish_seen_subcommand_from $ros2_commands $ros2_command_extensions" -a launch -d "Run a launch file"
$C -n "not __fish_seen_subcommand_from $ros2_commands $ros2_command_extensions" -a lifecycle -d "lifecycle related sub-commands"
$C -n "not __fish_seen_subcommand_from $ros2_commands $ros2_command_extensions" -a multicast -d "multicast related sub-commands"
$C -n "not __fish_seen_subcommand_from $ros2_commands $ros2_command_extensions" -a node -d "node related sub-commands"
$C -n "not __fish_seen_subcommand_from $ros2_commands $ros2_command_extensions" -a param -d "param related sub-commands"
$C -n "not __fish_seen_subcommand_from $ros2_commands $ros2_command_extensions" -a pkg -d "package related sub-commands"
$C -n "not __fish_seen_subcommand_from $ros2_commands $ros2_command_extensions" -a run -d "Run a package specific executable"
$C -n "not __fish_seen_subcommand_from $ros2_commands $ros2_command_extensions" -a security -d "security related sub-commands"
$C -n "not __fish_seen_subcommand_from $ros2_commands $ros2_command_extensions" -a service -d "service related sub-commands"
$C -n "not __fish_seen_subcommand_from $ros2_commands $ros2_command_extensions" -a topic -d "topic related sub-commands"
$C -n "not __fish_seen_subcommand_from $ros2_commands $ros2_command_extensions" -a wtf -d "Use `wtf` as alias to `doctor`"



# ros2 action -------------------------------------------------------------------------------------
# ros2 action --help
# usage: ros2 action [-h] Call `ros2 action <command> -h` for more detailed usage. ...
#
# Various action related sub-commands
#
# optional arguments:
#   -h, --help            show this help message and exit
#
# Commands:
#   info       Print information about an action
#   list       Output a list of action names
#   send_goal  Send an action goal
#
#   Call `ros2 action <command> -h` for more detailed usage.

set -l ros2_action_commands info list send_goal
set -l ros2_action_command_descriptions \
    "Print information about an action" \
    "Output a list of action names" \
    "Send an action goal"

for i in (seq (count $ros2_action_commands))
    set -l command $ros2_action_commands[$i]
    set -l description $ros2_action_command_descriptions[$i]
    $C -n "__fish_seen_subcommand_from action; and not __fish_seen_subcommand_from $ros2_action_commands" -a $command -d $description
end

# ros2 bag ----------------------------------------------------------------------------------------
# ros2 bag --help
# usage: ros2 bag [-h] Call `ros2 bag <command> -h` for more detailed usage. ...
#
# Various rosbag related sub-commands
#
# optional arguments:
#   -h, --help            show this help message and exit
#
# Commands:
#   info    Print information about a bag to the screen
#   play    Play back ROS data from a bag
#   record  Record ROS data to a bag
#
#   Call `ros2 bag <command> -h` for more detailed usage.


function __ros2_fish_print_directories_containing --argument-names f
    # %h is the directory name
    find . -type f -name "$f" -printf "%h\n" | sort -u
end

set -l ros2_bag_commands info play record
set -l ros2_bag_command_descriptions \
    "Print information about a bag to the screen" \
    "Play back ROS data from a bag" \
    "Record ROS data to a bag"


$C -n "__fish_seen_subcommand_from bag; and not __fish_seen_subcommand_from $ros2_bag_commands" -a info -d "Print information about a bag to the screen"
$C -n "__fish_seen_subcommand_from bag; and not __fish_seen_subcommand_from $ros2_bag_commands" -a play -d "Play back ROS data from a bag"
$C -n "__fish_seen_subcommand_from bag; and not __fish_seen_subcommand_from $ros2_bag_commands" -a record -d "Record ROS data to a bag"

# $C -n "__fish_seen_subcommand_with_subsubcommand bag info" -a "(__fish_print_files_in_subdirectories_with_extension bag)"
$C -n "__fish_seen_subcommand_with_subsubcommand bag info" -a "(__ros2_fish_print_directories_containing metadata.yaml)"

# $C -n "__fish_seen_subcommand_with_subsubcommand bag play" -a "(__fish_print_files_in_subdirectories_with_extension bag)"
$C -n "__fish_seen_subcommand_with_subsubcommand bag play" -a "(__ros2_fish_print_directories_containing metadata.yaml)"


# ros2 component ----------------------------------------------------------------------------------
# ros2 component --help
# usage: ros2 component [-h] Call `ros2 component <command> -h` for more detailed usage. ...
#
# Various component related sub-commands
#
# optional arguments:
#   -h, --help            show this help message and exit
#
# Commands:
#   list        Output a list of running containers and components
#   load        Load a component into a container node
#   standalone  Run a component into its own standalone container node
#   types       Output a list of components registered in the ament index
#   unload      Unload a component from a container node
#
#   Call `ros2 component <command> -h` for more detailed usage.

set -l ros2_component_commands list load standalone types unload
set -l ros2_component_command_descriptions \
    "Output a list of running containers and components" \
    "Load a component into a container node" \
    "Run a component into its own standalone container node" \
    "Output a list of components registered in the ament index" \
    "Unload a component from a container node"

for i in (seq (count $ros2_component_commands))
    set -l command $ros2_component_commands[$i]
    set -l description $ros2_component_command_descriptions[$i]
    $C -n "__fish_seen_subcommand_from component; and not __fish_seen_subcommand_from $ros2_component_commands" -a $command -d $description
end

# ros2 daemon -------------------------------------------------------------------------------------
# ros2 daemon --help
# usage: ros2 daemon [-h] Call `ros2 daemon <command> -h` for more detailed usage. ...
#
# Various daemon related sub-commands
#
# optional arguments:
#   -h, --help            show this help message and exit
#
# Commands:
#   start   Start the daemon if it isn't running
#   status  Output the status of the daemon
#   stop    Stop the daemon if it is running
#
#   Call `ros2 daemon <command> -h` for more detailed usage.

set -l ros2_daemon_commands start status stop
set -l ros2_daemon_command_descriptions \
    "Start the daemon if it isn't running" \
    "Output the status of the daemon" \
    "Stop the daemon if it is running"

for i in (seq (count $ros2_daemon_commands))
    set -l command $ros2_daemon_commands[$i]
    set -l description $ros2_daemon_command_descriptions[$i]
    $C -n "__fish_seen_subcommand_from daemon; and not __fish_seen_subcommand_from $ros2_daemon_commands" -a $command -d $description
end

# ros2 doctor -------------------------------------------------------------------------------------
# ros2 doctor --help
# usage: ros2 doctor [-h] [--report | --report-failed] [--include-warnings] Call `ros2 doctor <command> -h` for more detailed usage. ...
#
# Check ROS setup and other potential issues
#
# optional arguments:
#   -h, --help            show this help message and exit
#   --report, -r          Print all reports.
#   --report-failed, -rf  Print reports of failed checks only.
#   --include-warnings, -iw
#                         Include warnings as failed checks. Warnings are ignored by default.
#
# Commands:
#   hello  Check network connectivity between multiple hosts
#
#   Call `ros2 doctor <command> -h` for more detailed usage.

set -l ros2_doctor_commands hello
set -l ros2_doctor_command_descriptions \
    "Check network connectivity between multiple hosts"

for i in (seq (count $ros2_doctor_commands))
    set -l command $ros2_doctor_commands[$i]
    set -l description $ros2_doctor_command_descriptions[$i]
    $C -n "__fish_seen_subcommand_from doctor wtf; and not __fish_seen_subcommand_from $ros2_doctor_commands" -a $command -d $description
end

$C -n "__fish_seen_subcommand_from doctor" -s r -l report -d "Print all reports."
$C -n "__fish_seen_subcommand_from doctor" -o rf -l report-failed -d "Print reports of failed checks only."
$C -n "__fish_seen_subcommand_from doctor" -o iw -l include-warnings -d "Include warnings as failed checks. Warnings are ignored by default."

# ros2 interface ----------------------------------------------------------------------------------
# ros2 interface --help
# usage: ros2 interface [-h] Call `ros2 interface <command> -h` for more detailed usage. ...
#
# Show information about ROS interfaces
#
# optional arguments:
#   -h, --help            show this help message and exit
#
# Commands:
#   list      List all interface types available
#   package   Output a list of available interface types within one package
#   packages  Output a list of packages that provide interfaces
#   proto     Output an interface prototype
#   show      Output the interface definition
#
#   Call `ros2 interface <command> -h` for more detailed usage.

set -l ros2_interface_commands list package packages proto show
set -l ros2_interface_command_descriptions \
    "List all interface types available" \
    "Output a list of available interface types within one package" \
    "Output a list of packages that provide interfaces" \
    "Output an interface prototype" \
    "Output the interface definition"

for i in (seq (count $ros2_interface_commands))
    set -l command $ros2_interface_commands[$i]
    set -l description $ros2_interface_command_descriptions[$i]
    $C -n "__fish_seen_subcommand_from interface; and not __fish_seen_subcommand_from $ros2_interface_commands" -a $command -d $description
end

# ros2 launch -------------------------------------------------------------------------------------
$C -n "__fish_seen_subcommand_from launch" -s d -l debug -d "Put the launch system in debug mode, provides more verbose output"
$C -n "__fish_seen_subcommand_from launch" -s n -l noninteractive -d "Run the launch system non-interactively, with no terminal associated"
$C -n "__fish_seen_subcommand_from launch" -s p -l print -d "Print the launch description to the console without launching it."
$C -n "__fish_seen_subcommand_from launch" -s s -l show-args -d "Show arguments that may be given to the launch file."
$C -n "__fish_seen_subcommand_from launch" -s a -l show-all-subprocesses-output -d "Show all launched subprocesses' output by overriding their output configuration using the OVERRIDE_LAUNCH_PROCESS_OUTPUT envvar."

$C -n "__fish_seen_subcommand_from launch; and test (count (commandline -opc)) -eq 2" -a "(__fish_ros2_print_packages)"
$C -n "__fish_seen_subcommand_from launch; and contains (commandline -opc)[-1] (__fish_ros2_print_packages)" -a "(__fish_ros2_print_launch_filenames_in_package_with_desc (commandline -opc)[-1])"

$C -n "__fish_seen_subcommand_from launch; and test (count (commandline -opc)) -ge 4; and contains (commandline -opc)[4] (__fish_ros2_print_launch_filenames_in_package (commandline -opc)[3])" -a "(__fish_ros2_print_launch_file_args (commandline -opc)[3] (commandline -opc)[4])"

# ros2 lifecycle ----------------------------------------------------------------------------------
# ros2 lifecycle --help
# usage: ros2 lifecycle [-h] Call `ros2 lifecycle <command> -h` for more detailed usage. ...
#
# Various lifecycle related sub-commands
#
# optional arguments:
#   -h, --help            show this help message and exit
#
# Commands:
#   get    Get lifecycle state for one or more nodes
#   list   Output a list of available transitions
#   nodes  Output a list of nodes with lifecycle
#   set    Trigger lifecycle state transition
#
#   Call `ros2 lifecycle <command> -h` for more detailed usage.

set -l ros2_lifecycle_commands get list nodes set
set -l ros2_lifecycle_command_descriptions \
    "Get lifecycle state for one or more nodes" \
    "Output a list of available transitions" \
    "Output a list of nodes with lifecycle" \
    "Trigger lifecycle state transition"

for i in (seq (count $ros2_lifecycle_commands))
    set -l command $ros2_lifecycle_commands[$i]
    set -l description $ros2_lifecycle_command_descriptions[$i]
    $C -n "__fish_seen_subcommand_from lifecycle; and not __fish_seen_subcommand_from $ros2_lifecycle_commands" -a $command -d $description
end

$C -n "__fish_seen_subcommand_with_subsubcommand lifecycle set; and not __fish_ros2_cmd_in_array ($__fish_ros2 lifecycle nodes)" -a "($__fish_ros2 lifecycle nodes)"
$C -n "__fish_seen_subcommand_with_subsubcommand lifecycle get" -a "($__fish_ros2 lifecycle nodes)"
$C -n "__fish_seen_subcommand_with_subsubcommand lifecycle list" -a "($__fish_ros2 lifecycle nodes)"

$C -n "__fish_seen_subcommand_with_subsubcommand lifecycle set; and __fish_seen_nth_arg_from -1 ($__fish_ros2 lifecycle nodes)" -a "(__fish_ros2_print_available_lifecycle_transisions (commandline -opc)[-1])"

# ros2 multicast ----------------------------------------------------------------------------------
# ros2 multicast --help
# usage: ros2 multicast [-h] Call `ros2 multicast <command> -h` for more detailed usage. ...
#
# Various multicast related sub-commands
#
# optional arguments:
#   -h, --help            show this help message and exit
#
# Commands:
#   receive  Receive a single UDP multicast packet
#   send     Send a single UDP multicast packet
#
#   Call `ros2 multicast <command> -h` for more detailed usage.

set -l ros2_multicast_commands receive send
set -l ros2_multicast_command_descriptions \
    "Receive a single UDP multicast packet" \
    "Send a single UDP multicast packet"

for i in (seq (count $ros2_multicast_commands))
    set -l command $ros2_multicast_commands[$i]
    set -l description $ros2_multicast_command_descriptions[$i]
    $C -n "__fish_seen_subcommand_from multicast; and not __fish_seen_subcommand_from $ros2_multicast_commands" -a $command -d $description
end

# ros2 param --------------------------------------------------------------------------------------
# ros2 param --help
# usage: ros2 param [-h] Call `ros2 param <command> -h` for more detailed usage. ...
#
# Various param related sub-commands
#
# optional arguments:
#   -h, --help            show this help message and exit
#
# Commands:
#   delete    Delete parameter
#   describe  Show descriptive information about declared parameters
#   dump      Dump the parameters of a node to a yaml file
#   get       Get parameter
#   list      Output a list of available parameters
#   load      Load parameter file for a node
#   set       Set parameter
#
#   Call `ros2 param <command> -h` for more detailed usage.

set -l ros2_param_commands delete describe dump get list load set
set -l ros2_param_command_descriptions \
    "Delete parameter" \
    "Show descriptive information about declared parameters" \
    "Dump the parameters of a node to a yaml file" \
    "Get parameter" \
    "Output a list of available parameters" \
    "Load parameter file for a node" \
    "Set parameter"

for i in (seq (count $ros2_param_commands))
    set -l command $ros2_param_commands[$i]
    set -l description $ros2_param_command_descriptions[$i]
    $C -n "__fish_seen_subcommand_from param; and not __fish_seen_subcommand_from $ros2_param_commands" -a $command -d $description
end

# ros2 pkg ----------------------------------------------------------------------------------------
# ros2 pkg --help
# usage: ros2 pkg [-h] Call `ros2 pkg <command> -h` for more detailed usage. ...
#
# Various package related sub-commands
#
# optional arguments:
#   -h, --help            show this help message and exit
#
# Commands:
#   create       Create a new ROS2 package
#   executables  Output a list of package specific executables
#   list         Output a list of available packages
#   prefix       Output the prefix path of a package
#   xml          Output the XML of the package manifest or a specific tag
#
#   Call `ros2 pkg <command> -h` for more detailed usage.

set -l ros2_pkg_commands create executables list prefix xml
set -l ros2_pkg_command_descriptions \
    "Create a new ROS2 package" \
    "Output a list of package specific executables" \
    "Output a list of available packages" \
    "Output the prefix path of a package" \
    "Output the XML of the package manifest or a specific tag"

for i in (seq (count $ros2_pkg_commands))
    set -l command $ros2_pkg_commands[$i]
    set -l description $ros2_pkg_command_descriptions[$i]
    $C -n "__fish_seen_subcommand_from pkg; and not __fish_seen_subcommand_from $ros2_pkg_commands" -a $command -d $description
end

$C -n "__fish_seen_subcommand_with_subsubcommand pkg executables" -a "(__fish_ros2_print_packages)"
$C -n "__fish_seen_subcommand_with_subsubcommand pkg xml" -a "(__fish_ros2_print_packages)"
$C -n "__fish_seen_subcommand_with_subsubcommand pkg prefix" -a "(__fish_ros2_print_packages)"

# TODO: `ros2 pkg xml <package_name> --tag <tag_name>`
# $C -n "__fish_seen_subcommand_with_subsubcommand pkg xml" -a -s t -l tag -d "The XML tag to output (e.g. 'version')"


# ros2 run ----------------------------------------------------------------------------------------
# ros2 run --help
# usage: ros2 run [-h] [--prefix PREFIX] package_name executable_name ...
#
# Run a package specific executable
#
# positional arguments:
#   package_name     Name of the ROS package
#   executable_name  Name of the executable
#   argv             Pass arbitrary arguments to the executable
#
# optional arguments:
#   -h, --help       show this help message and exit
#   --prefix PREFIX  Prefix command, which should go before the executable. Command must be wrapped in quotes if it contains spaces (e.g. --prefix 'gdb -ex run --args').

$C -n "__fish_seen_subcommand_from run; and test (count (commandline -opc)) -eq 2" -a "(__fish_ros2_print_packages)"
$C -n "__fish_seen_subcommand_with_argument run" -a "(__fish_ros2_print_executables_in_package (commandline -opc)[3])"



# ros2 security -----------------------------------------------------------------------------------
# ros2 security --help
# usage: ros2 security [-h] Call `ros2 security <command> -h` for more detailed usage. ...
#
# Various security related sub-commands
#
# optional arguments:
#   -h, --help            show this help message and exit
#
# Commands:
#   create_key          Create key
#   create_keystore     Create keystore
#   create_permission   Create permission
#   generate_artifacts  Generate keys and permission files from a list of identities and policy files
#   generate_policy     Generate XML policy file from ROS graph data
#   list_keys           List keys
#
#   Call `ros2 security <command> -h` for more detailed usage.

set -l ros2_security_commands create_key create_keystore create_permission generate_artifacts generate_policy list_keys

set -l ros2_security_command_descriptions \
    "Create key" \
    "Create keystore" \
    "Create permission" \
    "Generate keys and permission files from a list of identities and policy files" \
    "Generate XML policy file from ROS graph data" \
    "List keys"

for i in (seq (count $ros2_security_commands))
    set -l command $ros2_security_commands[$i]
    set -l description $ros2_security_command_descriptions[$i]
    $C -n "__fish_seen_subcommand_from security; and not __fish_seen_subcommand_from $ros2_security_commands" -a $command -d $description
end

# ros2 service ------------------------------------------------------------------------------------
# ros2 service --help
# usage: ros2 service [-h] [--include-hidden-services] Call `ros2 service <command> -h` for more detailed usage. ...
#
# Various service related sub-commands
#
# optional arguments:
#   -h, --help            show this help message and exit
#   --include-hidden-services
#                         Consider hidden services as well
#
# Commands:
#   call  Call a service
#   find  Output a list of available services of a given type
#   list  Output a list of available services
#   type  Output a service's type
#
#   Call `ros2 service <command> -h` for more detailed usage.

set -l ros2_service_commands call find list type
set -l ros2_service_command_descriptions \
    "Call a service" \
    "Output a list of available services of a given type" \
    "Output a list of available services" \
    "Output a service's type"

for i in (seq (count $ros2_service_commands))
    set -l command $ros2_service_commands[$i]
    set -l description $ros2_service_command_descriptions[$i]
    $C -n "__fish_seen_subcommand_from service; and not __fish_seen_subcommand_from $ros2_service_commands" -a $command -d $description
end

$C -n "__fish_seen_subcommand_with_subsubcommand service call; and not __fish_ros2_cmd_in_array ($__fish_ros2 service list)" -a "(__fish_ros2_print_services)"
$C -n "__fish_seen_subcommand_with_subsubcommand service info" -a "(__fish_ros2_print_services)"

# Get the service type, assuming the last cmdline token is the service name
function __fish_ros2_get_service_type
    set -l cmd (commandline -poc)
    set -l service $cmd[-1]

    echo ($__fish_ros2 service type $service)
    return 0
end

$C -n "__fish_seen_subcommand_with_subsubcommand service call; and __fish_seen_nth_arg_from -1 ($__fish_ros2 service list)" -a "(__fish_ros2_get_service_type)"

# Check if ros2 service call is called with the last token on the command line being service type and the second to last one the service name
function __fish_seen_ros2_service_call_with_service_name_and_type
    if not __fish_seen_subcommand_with_subsubcommand service call
        return 1
    end

    set -l cmd (commandline -poc)
    if not contains -- $cmd[-2] ($__fish_ros2 service list)
        return 1
    end
    return 0
end

# Get the service proto, assuming the last cmdline token is the service type and the second to last one is the service name
function __fish_ros2_get_service_proto
    set -l cmd (commandline -poc)
    set -l service_type $cmd[-1]
    set -l service_type_proto ($__fish_ros2 interface proto --no-quotes $service_type)
    if not test -n "$service_type_proto"
        return 1
    end
    echo (proto_output_to_yaml $service_type_proto)
    return 0
end

$C -n __fish_seen_ros2_service_call_with_service_name_and_type -a "(__fish_ros2_get_service_proto)"

# ros2 topic --------------------------------------------------------------------------------------
# ros2 topic --help
# usage: ros2 topic [-h] [--include-hidden-topics] Call `ros2 topic <command> -h` for more detailed usage. ...
#
# Various topic related sub-commands
#
# optional arguments:
#   -h, --help            show this help message and exit
#   --include-hidden-topics
#                         Consider hidden topics as well
#
# Commands:
#   bw     Display bandwidth used by topic
#   delay  Display delay of topic from timestamp in header
#   echo   Output messages from a topic
#   find   Output a list of available topics of a given type
#   hz     Print the average publishing rate to screen
#   info   Print information about a topic
#   list   Output a list of available topics
#   pub    Publish a message to a topic
#   type   Print a topic's type
#
#   Call `ros2 topic <command> -h` for more detailed usage.

set -l ros2_topic_commands bw delay echo find hz info list pub type
set -l ros2_topic_command_descriptions \
    "Display bandwidth used by topic" \
    "Display delay of topic from timestamp in header" \
    "Output messages from a topic" \
    "Output a list of available topics of a given type" \
    "Print the average publishing rate to screen" \
    "Print information about a topic" \
    "Output a list of available topics" \
    "Publish a message to a topic" \
    "Print a topic's type"

for i in (seq (count $ros2_topic_commands))
    set -l command $ros2_topic_commands[$i]
    set -l description $ros2_topic_command_descriptions[$i]
    $C -n "__fish_seen_subcommand_from topic; and not __fish_seen_subcommand_from $ros2_topic_commands" -a $command -d $description
end

$C -n "__fish_seen_subcommand_with_subsubcommand topic echo" -a "(__fish_ros2_print_topics)"
$C -n "__fish_seen_subcommand_with_subsubcommand topic pub; and not __fish_ros2_cmd_in_array ($__fish_ros2 topic list)" -a "(__fish_ros2_print_topics)"
$C -n "__fish_seen_subcommand_with_subsubcommand topic hz" -a "(__fish_ros2_print_topics)"
$C -n "__fish_seen_subcommand_with_subsubcommand topic bw" -a "(__fish_ros2_print_topics)"
$C -n "__fish_seen_subcommand_with_subsubcommand topic info" -a "(__fish_ros2_print_topics)"
$C -n "__fish_seen_subcommand_with_subsubcommand topic type" -a "(__fish_ros2_print_topics)"

# Get the topic type, assuming the last cmdline token is the topic name
function __fish_ros2_get_topic_type
    set -l cmd (commandline -poc)
    set -l topic $cmd[-1]

    echo ($__fish_ros2 topic type $topic)
    return 0
end

$C -n "__fish_seen_subcommand_with_subsubcommand topic pub; and __fish_seen_nth_arg_from -1 ($__fish_ros2 topic list)" -a "(__fish_ros2_get_topic_type)"

# Check if ros2 topic pub is called with the last token on the command line being topic type and the second to last one the topic name
function __fish_seen_ros2_topic_pub_with_topic_name_and_type
    if not __fish_seen_subcommand_with_subsubcommand topic pub
        return 1
    end

    set -l cmd (commandline -poc)
    if not contains -- $cmd[-2] ($__fish_ros2 topic list)
        return 1
    end
    return 0
end

# Get the topic proto, assuming the last cmdline token is the topic type and the second to last one is the topic name
function __fish_ros2_get_topic_proto
    set -l cmd (commandline -poc)
    set -l topic_type $cmd[-1]
    set -l topic_type_proto ($__fish_ros2 interface proto --no-quotes $topic_type)
    if not test -n "$topic_type_proto"
        return 1
    end
    echo (proto_output_to_yaml $topic_type_proto)
    return 0
end

$C -n __fish_seen_ros2_topic_pub_with_topic_name_and_type -a "(__fish_ros2_get_topic_proto)"

# With the ros2 control library, it adds the following terminal commands:
# Currently supported commands are

#     ros2 control list_controllers
#     ros2 control list_controller_types
#     ros2 control list_hardware_components
#     ros2 control list_hardware_interfaces
#     ros2 control load_controller
#     ros2 control reload_controller_libraries
#     ros2 control set_controller_state
#     ros2 control switch_controllers
#     ros2 control unload_controller
#     ros2 control view_controller_chains

# However as they are not support (yet) in this repo, if you try to autocomplete these, it will change the whole line to something else.

set -l ros2_control_verbs \
    list_controllers \
    list_controller_types \
    list_hardware_components \
    list_hardware_interfaces \
    load_controller \
    reload_controller_libraries \
    set_controller_state \
    switch_controllers \
    unload_controller \
    view_controller_chains


# TODO: check if ros2_control is installed before adding suggestions
$C -n "not __fish_seen_subcommand_from $ros2_commands control" -a control -d "ros2_control (extension)"
$C -n "__fish_seen_subcommand_from control" -a "$ros2_control_verbs"

# for verb in $ros2_control_verbs
#     $C -n "__fish_seen_subcommand_from control" -a $verb
# end
