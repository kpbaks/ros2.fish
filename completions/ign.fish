# set -l C complete --command ign
#
# $C -f # disable file completion
#
# set -l commands (ign --commands)
#
#
#
# # The 'ign' command provides a command line interface to the ignition tools.
# #
# #   ign <command> [options]
# #
# # List of available commands:
# #
# #   help:          Print this help text.
# #   gui:           Launch graphical interfaces.
# #   gazebo:        Run and manage Gazebo.
# #   fuel:          Manage simulation resources.
# #   msg:           Print information about messages.
# #   sdf:           Utilities for SDF files.
# #   plugin:        Print information about plugins.
# #   model:         Print information about models.
# #   topic:         Print information about topics.
# #   service:       Print information about services.
# #   log:           Record or playback topics.
# #
# # Options:
# #
# #   --force-version <VERSION>  Use a specific library version.
# #   --versions                 Show the available versions.
# #   --commands                 Show the available commands.
# # Use 'ign help <command>' to print help for a command.
#
#
# $C -l force-version -r -d "Use a specific library version."
# $C -l versions -d "Show the available versions."
# $C -l commands -d "Show the available commands."
#
# $C -n "not __fish_seen_subcommand_from $commands" -a help -d "Print help information"
# $C -n "not __fish_seen_subcommand_from $commands" -a gui -d "Launch graphical interfaces."
# $C -n "not __fish_seen_subcommand_from $commands" -a gazebo -d "Run and manage Gazebo."
# $C -n "not __fish_seen_subcommand_from $commands" -a fuel -d "Manage simulation resources."
# $C -n "not __fish_seen_subcommand_from $commands" -a msg -d "Print information about messages."
# $C -n "not __fish_seen_subcommand_from $commands" -a sdf -d "Utilities for SDF files."
# $C -n "not __fish_seen_subcommand_from $commands" -a plugin -d "Print information about plugins."
# $C -n "not __fish_seen_subcommand_from $commands" -a model -d "Print information about models."
# $C -n "not __fish_seen_subcommand_from $commands" -a topic -d "Print information about topics."
# $C -n "not __fish_seen_subcommand_from $commands" -a service -d "Print information about services."
# $C -n "not __fish_seen_subcommand_from $commands" -a log -d "Record or playback topics."
