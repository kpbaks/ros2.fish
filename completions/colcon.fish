set -l c complete --command colcon
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

# colcon build ------------------------------------------------------------------------------------
# colcon build -h
# usage: colcon build [-h] [--build-base BUILD_BASE] [--install-base INSTALL_BASE] [--merge-install] [--symlink-install] [--test-result-base TEST_RESULT_BASE]
#                     [--continue-on-error] [--executor {parallel,sequential}] [--parallel-workers NUMBER] [--event-handlers [name1+ [name2- ...]]] [--ignore-user-meta]
#                     [--metas [PATH [PATH ...]]] [--base-paths [PATH [PATH ...]]] [--packages-ignore [PKG_NAME [PKG_NAME ...]]]
#                     [--packages-ignore-regex [PATTERN [PATTERN ...]]] [--paths [PATH [PATH ...]]] [--packages-up-to [PKG_NAME [PKG_NAME ...]]]
#                     [--packages-up-to-regex [PATTERN [PATTERN ...]]] [--packages-above [PKG_NAME [PKG_NAME ...]]]
#                     [--packages-above-and-dependencies [PKG_NAME [PKG_NAME ...]]] [--packages-above-depth DEPTH [PKG_NAME ...]]
#                     [--packages-select-by-dep [PKG_NAME [PKG_NAME ...]]] [--packages-skip-by-dep [PKG_NAME [PKG_NAME ...]]]
#                     [--packages-skip-up-to [PKG_NAME [PKG_NAME ...]]]
#                     [--packages-select-build-failed | --packages-skip-build-finished | --packages-select-test-failures | --packages-skip-test-passed]
#                     [--packages-select [PKG_NAME [PKG_NAME ...]]] [--packages-skip [PKG_NAME [PKG_NAME ...]]] [--packages-select-regex [PATTERN [PATTERN ...]]]
#                     [--packages-skip-regex [PATTERN [PATTERN ...]]] [--packages-start PKG_NAME] [--packages-end PKG_NAME] [--cmake-args [* [* ...]]]
#                     [--cmake-target CMAKE_TARGET] [--cmake-target-skip-unavailable] [--cmake-clean-cache] [--cmake-clean-first] [--cmake-force-configure]
#                     [--ament-cmake-args [* [* ...]]] [--catkin-cmake-args [* [* ...]]] [--catkin-skip-building-tests] [--mixin-files [FILE [FILE ...]]]
#                     [--mixin [mixin1 [mixin2 ...]]]
#
# Build a set of packages.
#
# optional arguments:
#   -h, --help            show this help message and exit
#   --build-base BUILD_BASE
#                         The base path for all build directories (default: build)
#   --install-base INSTALL_BASE
#                         The base path for all install prefixes (default: install)
#   --merge-install       Merge all install prefixes into a single location
#   --symlink-install     Use symlinks instead of copying files where possible
#   --test-result-base TEST_RESULT_BASE
#                         The base path for all test results (default: --build-base)
#   --continue-on-error   Continue other packages when a package fails to build (packages recursively depending on the failed package are skipped)
#
# Executor arguments:
#   --executor {parallel,sequential}
#                         The executor to process all packages (default: parallel)
#                         * parallel: Process multiple packages in parallel
#                         * sequential: Process one package at a time
#   --parallel-workers NUMBER
#                         The maximum number of packages to process in parallel (default: 8)
#
# Event handler arguments:
#   --event-handlers [name1+ [name2- ...]]
#                         Enable (+) or disable (-) event handlers (default: compile_commands+ console_cohesion- console_direct- console_package_list- console_start_end+
#                         console_stderr+ desktop_notification+ event_log+ log+ log_command+ status+ store_result+ summary+ terminal_title+)
#                         * compile_commands: Generate a `compile_commands.json` file for the whole workspace
#                         * console_cohesion: Pass task output at once to stdout
#                         * console_direct: Pass output directly to stdout/err
#                         * console_package_list: Output list of queued job names
#                         * console_start_end: Output task name on start/end
#                         * console_stderr: Output all stderr of a task at once
#                         * desktop_notification: Desktop notification of the summary
#                         * event_log: Log all events to a global log file
#                         * log: Output task specific log files
#                         * log_command: Log a 'debug' message for each command
#                         * status: Continuously update a status line
#                         * store_result: Persist the result of a job in a file in its build directory
#                         * summary: Output summary of all tasks
#                         * terminal_title: Show status in the terminal title
#
# Discovery arguments:
#   --ignore-user-meta    Ignore `*.meta` files in the user config directory `/home/kristoffer/.var/distrobox/home/ros2-foxy/.colcon/metadata`
#   --metas [PATH [PATH ...]]
#                         The directories containing a `colcon.meta` file or paths to arbitrary files containing the same meta information (default: ./colcon.meta)
#   --base-paths [PATH [PATH ...]]
#                         The base paths to recursively crawl for packages (default: .)
#   --packages-ignore [PKG_NAME [PKG_NAME ...]]
#                         Ignore packages as if they were not discovered
#   --packages-ignore-regex [PATTERN [PATTERN ...]]
#                         Ignore packages where any of the patterns match the package name
#   --paths [PATH [PATH ...]]
#                         The paths to check for a package. Use shell wildcards (e.g. `src/*`) to select all direct subdirectories
#
# Package selection arguments:
#   --packages-up-to [PKG_NAME [PKG_NAME ...]]
#                         Only process a subset of packages and their recursive dependencies
#   --packages-up-to-regex [PATTERN [PATTERN ...]]
#                         Only process a subset of packages and their recursive dependencies, where any of the patterns match the package name
#   --packages-above [PKG_NAME [PKG_NAME ...]]
#                         Only process a subset of packages and packages which recursively depend on them
#   --packages-above-and-dependencies [PKG_NAME [PKG_NAME ...]]
#                         Only process a subset of packages and packages which recursively depend on them including all their recursive dependencies
#   --packages-above-depth DEPTH [PKG_NAME ...]
#                         Only process a subset of packages and packages which recursively depend on them up to a given depth
#   --packages-select-by-dep [PKG_NAME [PKG_NAME ...]]
#                         Only process packages which (recursively) depend on this
#   --packages-skip-by-dep [PKG_NAME [PKG_NAME ...]]
#                         Skip packages which (recursively) depend on this
#   --packages-skip-up-to [PKG_NAME [PKG_NAME ...]]
#                         Skip a subset of packages and their recursive dependencies
#   --packages-select-build-failed
#                         Only process a subset of packages which have failed to build previously (aborted packages are not considered errors)
#   --packages-skip-build-finished
#                         Skip a set of packages which have finished to build previously
#   --packages-select-test-failures
#                         Only process a subset of packages which had test failures previously
#   --packages-skip-test-passed
#                         Skip a set of packages which had no test failures previously
#   --packages-select [PKG_NAME [PKG_NAME ...]]
#                         Only process a subset of packages
#   --packages-skip [PKG_NAME [PKG_NAME ...]]
#                         Skip a set of packages
#   --packages-select-regex [PATTERN [PATTERN ...]]
#                         Only process a subset of packages where any of the patterns match the package name
#   --packages-skip-regex [PATTERN [PATTERN ...]]
#                         Skip a set of packages where any of the patterns match the package name
#   --packages-start PKG_NAME
#                         Skip packages before this in flat topological ordering
#   --packages-end PKG_NAME
#                         Skip packages after this in flat topological ordering
#
# Arguments for 'cmake' packages:
#   --cmake-args [* [* ...]]
#                         Pass arguments to CMake projects. Arguments matching other options must be prefixed by a space,
#                         e.g. --cmake-args " --help" (stdout might not be shown by default, e.g. add `--event-handlers console_cohesion+`)
#   --cmake-target CMAKE_TARGET
#                         Build a specific target instead of the default target
#   --cmake-target-skip-unavailable
#                         Skip building packages which don't have the target passed to --cmake-target
#   --cmake-clean-cache   Remove CMake cache before the build (implicitly forcing CMake configure step)
#   --cmake-clean-first   Build target 'clean' first, then build (to only clean use '--cmake-target clean')
#   --cmake-force-configure
#                         Force CMake configure step
#
# Arguments for 'ros.ament_cmake' packages:
#   --ament-cmake-args [* [* ...]]
#                         Pass arguments to 'ament_cmake' packages. Arguments matching other options must be prefixed by a space
#
# Arguments for 'ros.catkin' packages:
#   --catkin-cmake-args [* [* ...]]
#                         Pass arguments to 'catkin' packages. Arguments matching other options must be prefixed by a space
#   --catkin-skip-building-tests
#                         By default the 'tests' target of 'catkin' packages is invoked. If running 'colcon test' later isn't intended this can be skipped
#
# Mixin predefined sets of command line parameters:
#   --mixin-files [FILE [FILE ...]]
#                         Additional files providing mixins
#   --mixin [mixin1 [mixin2 ...]]
#                         No mixins are available for this verb

complete -c colcon -n '__fish_seen_subcommand_from build' -l cmake-args -d 'Pass arguments to CMake projects. Arguments matching other options must be prefixed by a space, e.g. --cmake-args " --help" (stdout might not be shown by default, e.g. add `--event-handlers console_cohesion+`)'

# colcon build ------------------------------------------------------------------------------------
# Build-related options
complete -c colcon -n '__fish_seen_subcommand_from build' -l build-base -d 'The base path for all build directories (default: build)'
complete -c colcon -n '__fish_seen_subcommand_from build' -l install-base -d 'The base path for all install prefixes (default: install)'
complete -c colcon -n '__fish_seen_subcommand_from build' -l merge-install -d 'Merge all install prefixes into a single location'
complete -c colcon -n '__fish_seen_subcommand_from build' -l symlink-install -d 'Use symlinks instead of copying files where possible'
complete -c colcon -n '__fish_seen_subcommand_from build' -l test-result-base -d 'The base path for all test results (default: --build-base)'
complete -c colcon -n '__fish_seen_subcommand_from build' -l continue-on-error -d 'Continue other packages when a package fails to build'

# Executor options
complete -c colcon -n '__fish_seen_subcommand_from build' -l executor -a 'parallel sequential' -d 'The executor to process all packages (default: parallel)'
complete -c colcon -n '__fish_seen_subcommand_from build' -l parallel-workers -d 'The maximum number of packages to process in parallel (default: 8)'

# Event handler options
complete -c colcon -n '__fish_seen_subcommand_from build' -l event-handlers -d 'Enable (+) or disable (-) event handlers (e.g., console_stderr+, log-, summary+)'

# Discovery options
complete -c colcon -n '__fish_seen_subcommand_from build' -l ignore-user-meta -d 'Ignore *.meta files in the user config directory'
complete -c colcon -n '__fish_seen_subcommand_from build' -l metas -d 'The directories containing a colcon.meta file or paths to files containing the same meta information'
complete -c colcon -n '__fish_seen_subcommand_from build' -l base-paths -d 'The base paths to recursively crawl for packages (default: .)'
complete -c colcon -n '__fish_seen_subcommand_from build' -l packages-ignore -d 'Ignore packages as if they were not discovered'
complete -c colcon -n '__fish_seen_subcommand_from build' -l packages-ignore-regex -d 'Ignore packages where any of the patterns match the package name'

# Package selection options
complete -c colcon -n '__fish_seen_subcommand_from build' -l packages-up-to -d 'Only process a subset of packages and their recursive dependencies'
complete -c colcon -n '__fish_seen_subcommand_from build' -l packages-above -d 'Only process a subset of packages and packages which recursively depend on them'
complete -c colcon -n '__fish_seen_subcommand_from build' -l packages-select -d 'Only process a subset of packages'
complete -c colcon -n '__fish_seen_subcommand_from build' -l packages-skip -d 'Skip a set of packages'
complete -c colcon -n '__fish_seen_subcommand_from build' -l packages-select-regex -d 'Only process packages where any of the patterns match the package name'
complete -c colcon -n '__fish_seen_subcommand_from build' -l packages-skip-regex -d 'Skip packages where any of the patterns match the package name'

# CMake and ROS options
complete -c colcon -n '__fish_seen_subcommand_from build' -l cmake-target -d 'Build a specific target instead of the default target'
complete -c colcon -n '__fish_seen_subcommand_from build' -l cmake-clean-cache -d 'Remove CMake cache before the build (forcing CMake configure step)'
complete -c colcon -n '__fish_seen_subcommand_from build' -l cmake-force-configure -d 'Force CMake configure step'
complete -c colcon -n '__fish_seen_subcommand_from build' -l ament-cmake-args -d 'Pass arguments to ament_cmake packages'
complete -c colcon -n '__fish_seen_subcommand_from build' -l catkin-cmake-args -d 'Pass arguments to catkin packages'
complete -c colcon -n '__fish_seen_subcommand_from build' -l catkin-skip-building-tests -d 'Skip building tests for catkin packages'

# Mixin options
complete -c colcon -n '__fish_seen_subcommand_from build' -l mixin-files -d 'Additional files providing mixins'
complete -c colcon -n '__fish_seen_subcommand_from build' -l mixin -d 'No mixins are available for this verb'

# General help for 'colcon build'
complete -c colcon -n '__fish_seen_subcommand_from build' -s h -l help -d 'Show help information for build command'

# Enable directory name completion inside ./src for 'colcon build', only if ./src/ exists
if test -d ./src
    complete -c colcon -n '__fish_seen_subcommand_from build; and __fish_seen_argument --packages-select' -f -a '(basename -a (find ./src -maxdepth 1 -type d -exec basename {} \;))'
    complete -c colcon -n '__fish_seen_subcommand_from build; and __fish_seen_argument --packages-up-to' -f -a '(basename -a (find ./src -maxdepth 1 -type d -exec basename {} \;))'
    complete -c colcon -n '__fish_seen_subcommand_from build; and __fish_seen_argument --packages-ignore' -f -a '(basename -a (find ./src -maxdepth 1 -type d -exec basename {} \;))'
    complete -c colcon -n '__fish_seen_subcommand_from build; and __fish_seen_argument --paths' -f -a '(basename -a (find ./src -maxdepth 1 -type d -exec basename {} \;))'
end

