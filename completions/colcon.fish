complete -c colcon -f # disable file completion

# colcon -h
# usage: colcon [-h] [--log-base LOG_BASE] [--log-level LOG_LEVEL] {build,extension-points,extensions,graph,info,list,metadata,mixin,test,test-result,version-check} ...
#
# optional arguments:
#   -h, --help            show this help message and exit
#   --log-base LOG_BASE   The base path for all log directories (default: ./log, to disable: /dev/null)
#   --log-level LOG_LEVEL
#                         Set log level for the console output, either by numeric or string value (default: warning)
#
# colcon verbs:
#   build                 Build a set of packages
#   extension-points      List extension points
#   extensions            List extensions
#   graph                 Generate a visual representation of the dependency graph
#   info                  Package information
#   list                  List packages, optionally in topological ordering
#   metadata              Manage metadata of packages
#   mixin                 Manage CLI mixins
#   test                  Test a set of packages
#   test-result           Show the test results generated when testing a set of
#                         packages
#   version-check         Compare local package versions with PyPI
#
#   {build,extension-points,extensions,graph,info,list,metadata,mixin,test,test-result,version-check}
#                         call `colcon VERB -h` for specific help
#
# Environment variables:
#   CMAKE_COMMAND         The full path to the CMake executable
#   COLCON_ALL_SHELLS     Flag to enable all shell extensions
#   COLCON_COMPLETION_LOGFILE
#                         Set the logfile for completion time
#   COLCON_DEFAULTS_FILE  Set path to the yaml file containing the default values
#                         for the command line arguments (default:
#                         $COLCON_HOME/defaults.yaml)
#   COLCON_DEFAULT_EXECUTOR
#                         Select the default executor extension
#   COLCON_EXTENSION_BLOCKLIST
#                         Block extensions which should not be used
#   COLCON_HOME           Set the configuration directory (default: ~/.colcon)
#   COLCON_LOG_LEVEL      Set the log level (debug|10, info|20, warn|30,
#                         error|40, critical|50, or any other positive numeric
#                         value)
#   COLCON_MIXIN_PATH     Provide additional directories to look for mixin files.
#                         Separate individual directories with colons.
#   COLCON_WARNINGS       Set the warnings filter similar to PYTHONWARNINGS
#                         except that the module entry is implicitly set to
#                         'colcon.*'
#   CTEST_COMMAND         The full path to the CTest executable
#   POWERSHELL_COMMAND    The full path to the PowerShell executable
#
# For more help and usage tips, see https://colcon.readthedocs.io

complete -c colcon -s h -l help -d "show help information"
complete -c colcon -l log-base -d "The base path for all log directories"
complete -c colcon -l log-level -d "Set log level for the console output, either by numeric or string value"

set -l colcon_verbs build extension-points extensions graph info list metadata mixin test test-result version-check

set -l colcon_verb_descriptions \
    "Build a set of packages" \
    "List extension points" \
    "List extensions" \
    "Generate a visual representation of the dependency graph" \
    "Package information" \
    "List packages, optionally in topological ordering" \
    "Manage metadata of packages" \
    "Manage CLI mixins" \
    "Test a set of packages" \
    "Show the test results generated when testing a set of packages" \
    "Compare local package versions with PyPI"

for i in (seq (count $colcon_verb_descriptions))
    set -l verb $colcon_verbs[$i]
    set -l description $colcon_verb_descriptions[$i]
    complete -c colcon -n "not __fish_seen_subcommand_from $colcon_verbs" -a $verb -d $description
end
