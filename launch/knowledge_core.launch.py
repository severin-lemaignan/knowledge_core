# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_pal_configuration(pkg, node, ld=None, cmdline_passthrough_args=None):
    """
    Get the configuration for a node from the PAL configuration files.

    :param pkg: The package name
    :param node: The node name
    :param ld: The launch description to log messages to. If None, no messages
    are logged.
    :param cmdline_passthrough_args: A list of arguments that will be added as
    command-line launch arguments

    :return: A dictionary with the parameters, remappings and arguments
    """
    import yaml
    import ament_index_python as aip
    from launch.actions import LogInfo

    import collections.abc

    # code for recursive dictionary update
    # taken from https://stackoverflow.com/a/3233356
    def update(d, u):
        for k, v in u.items():
            if isinstance(v, collections.abc.Mapping):
                d[k] = update(d.get(k, {}), v)
            else:
                d[k] = v
        return d

    cfg_srcs_pkgs = aip.get_resources(f"pal_configuration.{pkg}")

    cfg_srcs = {}
    for pkg, _ in cfg_srcs_pkgs.items():
        cfg_files, _ = aip.get_resource(f"pal_configuration.{pkg}", pkg)
        for cfg_file in cfg_files.split("\n"):
            share_path = aip.get_package_share_path(pkg)
            path = share_path / cfg_file
            if not path.exists():
                if ld:
                    ld.add_action(LogInfo(msg=f"WARNING: configuration file {path} does not exist."
                                          " Skipping it."))
                continue
            if path.name in cfg_srcs:
                if ld:
                    ld.add_action(LogInfo(msg="WARNING: two packages provide the same"
                                          f" configuration {path.name} for {pkg}:"
                                          f" {cfg_srcs[path.name]} and {path}. Skipping {path}"))
                continue
            cfg_srcs[path.name] = path

    config = {}
    for cfg_file in sorted(cfg_srcs.keys()):
        with open(cfg_srcs[cfg_file], 'r') as f:
            config = update(config, yaml.load(f, yaml.Loader))

    if not config:
        return {"parameters": [], "remappings": [], "arguments": []}

    node_fqn = None
    for k in config.keys():
        if k.split("/")[-1] == node:
            node_fqn = k
            break

    if not node_fqn:
        if ld:
            ld.add_action(LogInfo(msg="ERROR: configuration files found, but"
                                  f" node {node} has no entry!\nI looked into the following"
                                  " configuration files:"
                                  f" {[str(p) for k, p in cfg_srcs.items()]}\n"
                                  " Returning empty parameters/remappings/arguments"))
        return {"parameters": [], "remappings": [], "arguments": []}

    if "ros__parameters" in config[node_fqn] \
       and not isinstance(config[node_fqn]["ros__parameters"], dict):
        config[node_fqn]["ros__parameters"] = {}
        print("ERROR: 'ros__parameters' field in configuration"
              f" for node {node} must be a _dict_ of parameters."
              " (a common mistake is incorrect prefixing of parameters"
              " with hyphens, turning the dict into a list of dicts).")

    if "remappings" in config[node_fqn] \
       and not isinstance(config[node_fqn]["remappings"], dict):
        config[node_fqn]["remappings"] = {}
        print("ERROR: 'remappings' field in configuration"
              f" for node {node} must be a _dict_ of remappings."
              " (a common mistake is incorrect prefixing of remappings"
              " with hyphens, turning the dict into a list of dicts).")

    if "arguments" in config[node_fqn] \
       and not isinstance(config[node_fqn]["arguments"], dict):
        config[node_fqn]["arguments"] = []
        print("ERROR: 'arguments' field in configuration"
              f" for node {node} must be a _list_ of arguments"
              " to be passed to the node. Ignoring it.")

    if cmdline_passthrough_args:
        if ld is None:
            raise ValueError(
                "cmdline_passthrough_args can only be used if 'ld' is not None")

        for arg in cmdline_passthrough_args:
            default = config[node_fqn].setdefault(
                "ros__parameters", {}).get(arg)
            if default is None:
                print(f"WARNING: no default value defined for cmdline arguments <{arg}>. "
                      "As such, it is mandatory to set this argument when launching the node. "
                      "Consider adding a default value in the configuration file of the node.")

            ld.add_action(DeclareLaunchArgument(arg, default_value=default))
            config[node_fqn]["ros__parameters"][arg] = LaunchConfiguration(arg)

    res = {"parameters": [{k: v} for k, v in config[node_fqn].setdefault("ros__parameters", {})
                          .items()],
           "remappings": config[node_fqn].setdefault("remappings", {}).items(),
           "arguments": config[node_fqn].setdefault("arguments", []),
           }

    if ld:
        ld.add_action(
            LogInfo(msg=f"Loaded configuration for <{node}> "
                    f"from {[str(p) for k, p in cfg_srcs.items()]}"))
        if res['parameters']:
            ld.add_action(LogInfo(msg="Parameters: " +
                                  '\n - '.join([f"{k}: {v}" for d in res['parameters']
                                                for k, v in d.items()])))
        if res['remappings']:
            ld.add_action(LogInfo(msg="Remappings: " +
                                  '\n - '.join([f"{a} -> {b}" for a, b in res['remappings']])))
        if res['arguments']:
            ld.add_action(LogInfo(msg="Arguments: " +
                                  '\n - '.join(res['arguments'])))

    return res


def generate_launch_description():

    ld = LaunchDescription()

    config = get_pal_configuration(
        pkg='knowledge_core',
        node='knowledge_core',
        ld=ld,
        cmdline_passthrough_args=['default_kb'])

    knowledge_core_node = Node(
        package='knowledge_core',
        executable='knowledge_core',
        namespace='kb',
        name='knowledge_core',
        parameters=config["parameters"],
        remappings=config["remappings"],
        arguments=config["arguments"],
        output='both',
        emulate_tty=True,
    )

    ld.add_action(knowledge_core_node)

    return ld
