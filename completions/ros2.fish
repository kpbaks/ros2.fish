
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


complete -c ros2 -f # disable file completion

complete -c ros2 -s h -l help -d "show this help message and exit"

set -l ros2_commands action bag component daemon doctor interface launch lifecycle multicast node param pkg run security service topic wtf

complete -c ros2 -n "not __fish_seen_subcommand_from $ros2_commands" -a action -d "Various action related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros2_commands" -a bag -d "Various rosbag related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros2_commands" -a component -d "Various component related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros2_commands" -a daemon -d "Various daemon related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros2_commands" -a doctor -d "Check ROS setup and other potential issues"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros2_commands" -a interface -d "Show information about ROS interfaces"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros2_commands" -a launch -d "Run a launch file"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros2_commands" -a lifecycle -d "Various lifecycle related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros2_commands" -a multicast -d "Various multicast related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros2_commands" -a node -d "Various node related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros2_commands" -a param -d "Various param related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros2_commands" -a pkg -d "Various package related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros2_commands" -a run -d "Run a package specific executable"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros2_commands" -a security -d "Various security related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros2_commands" -a service -d "Various service related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros2_commands" -a topic -d "Various topic related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros2_commands" -a wtf -d "Use `wtf` as alias to `doctor`"



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
    complete -c ros2 -n "__fish_seen_subcommand_from action; and not __fish_seen_subcommand_from $ros2_action_commands" -a $command -d $description
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


set -l ros2_bag_commands info play record
set -l ros2_bag_command_descriptions \
    "Print information about a bag to the screen" \
    "Play back ROS data from a bag" \
    "Record ROS data to a bag"

for i in (seq (count $ros2_bag_commands))
    set -l command $ros2_bag_commands[$i]
    set -l description $ros2_bag_command_descriptions[$i]
    complete -c ros2 -n "__fish_seen_subcommand_from bag; and not __fish_seen_subcommand_from $ros2_bag_commands" -a $command -d $description
end

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
    complete -c ros2 -n "__fish_seen_subcommand_from component; and not __fish_seen_subcommand_from $ros2_component_commands" -a $command -d $description
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
    complete -c ros2 -n "__fish_seen_subcommand_from daemon; and not __fish_seen_subcommand_from $ros2_daemon_commands" -a $command -d $description
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
    complete -c ros2 -n "__fish_seen_subcommand_from doctor wtf; and not __fish_seen_subcommand_from $ros2_doctor_commands" -a $command -d $description
end

complete -c ros2 -n "__fish_seen_subcommand_from doctor" -s r -l report -d "Print all reports."
complete -c ros2 -n "__fish_seen_subcommand_from doctor" -o rf -l report-failed -d "Print reports of failed checks only."
complete -c ros2 -n "__fish_seen_subcommand_from doctor" -o iw -l include-warnings -d "Include warnings as failed checks. Warnings are ignored by default."

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
    complete -c ros2 -n "__fish_seen_subcommand_from interface; and not __fish_seen_subcommand_from $ros2_interface_commands" -a $command -d $description
end

# ros2 launch -------------------------------------------------------------------------------------


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
    complete -c ros2 -n "__fish_seen_subcommand_from lifecycle; and not __fish_seen_subcommand_from $ros2_lifecycle_commands" -a $command -d $description
end

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
    complete -c ros2 -n "__fish_seen_subcommand_from multicast; and not __fish_seen_subcommand_from $ros2_multicast_commands" -a $command -d $description
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
    complete -c ros2 -n "__fish_seen_subcommand_from param; and not __fish_seen_subcommand_from $ros2_param_commands" -a $command -d $description
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
    complete -c ros2 -n "__fish_seen_subcommand_from pkg; and not __fish_seen_subcommand_from $ros2_pkg_commands" -a $command -d $description
end

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
    complete -c ros2 -n "__fish_seen_subcommand_from security; and not __fish_seen_subcommand_from $ros2_security_commands" -a $command -d $description
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
    complete -c ros2 -n "__fish_seen_subcommand_from service; and not __fish_seen_subcommand_from $ros2_service_commands" -a $command -d $description
end

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
    complete -c ros2 -n "__fish_seen_subcommand_from topic; and not __fish_seen_subcommand_from $ros2_topic_commands" -a $command -d $description
end
