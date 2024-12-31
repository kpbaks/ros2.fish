# ros2.fish
Integrate ROS2 with the fish-shell, and add interactive goodies to make you robotics workflow a little more fluid.

## Requirements

- [fish ^3.6.0](https://github.com/fish-shell/fish-shell/releases/tag/3.6.0) enhanced the capabilities of `abbr` which this plugin makes use of.

## Installation

Using [fisher](https://github.com/jorgebucaran/fisher)

```sh
fisher install kpbaks/ros2.fish
```
In case to have a souerce installation, add the `ROS2_PATH` environment variable to the path of your ros installation at the end of the configuration file of your terminal.

example: of a source instalation of ROS2 rolling
```
ros2_rolling
├── build
├── install
├── log
└── src
```
```sh
# ROS2
export ROS2_PATH=~/ros2_rolling
```
