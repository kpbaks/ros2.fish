
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


complete -c ros2 -s h -l help -d "show this help message and exit"

set -l ros2_commands action bag component daemon doctor interface launch lifecycle multicast node param pkg run security service topic wtf

complete -c ros2 -n "not __fish_seen_subcommand_from $ros3_commands" -a action -d "Various action related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros3_commands" -a bag -d "Various rosbag related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros3_commands" -a component -d "Various component related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros3_commands" -a daemon -d "Various daemon related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros3_commands" -a doctor -d "Check ROS setup and other potential issues"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros3_commands" -a interface -d "Show information about ROS interfaces"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros3_commands" -a launch -d "Run a launch file"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros3_commands" -a lifecycle -d "Various lifecycle related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros3_commands" -a multicast -d "Various multicast related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros3_commands" -a node -d "Various node related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros3_commands" -a param -d "Various param related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros3_commands" -a pkg -d "Various package related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros3_commands" -a run -d "Run a package specific executable"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros3_commands" -a security -d "Various security related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros3_commands" -a service -d "Various service related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros3_commands" -a topic -d "Various topic related sub-commands"
complete -c ros2 -n "not __fish_seen_subcommand_from $ros3_commands" -a wtf -d "Use `wtf` as alias to `doctor`"
