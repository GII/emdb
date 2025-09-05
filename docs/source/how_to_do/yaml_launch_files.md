# How to configure an experiment

In this section, we will provide an explanation in how to use the YAML and launch files in order to configure our experiments.

These files appear in other How To Do sections, but here we will do a complete explanation of both.

## YAML files

The YAML file defines the parameters of the experiment that determine the behavior of the cognitive architecture.
There are two files to provide: The commander configuration file and the 

### Commander configuration file

This file, commander.yaml, is located in the core package, in the config directory (core/config). Here it's possible to decide the number of execution nodes that the commander is going to create and the number of threads of each one.
By default, there are 5 execution nodes with 1 thread for each one:
```
Commander:
    ExecutionNode:
        -
            threads: 1
        -
            threads: 1
        -
            threads: 1
        -
            threads: 1
        -
            threads: 1

```

### Experiment configuration file

This file contains the startup configuration that the commander will apply. It includes configurations for the cognitive processes to launch, default node connectors and the initial nodes in the architecture. Additionally, it can contain experiment-specific configurations such as the required for the simulators.

**Cognitive processes:**

We can select the cognitive processes to execute and define their parameters. 

Currently, we only have one cognitive process: the main loop, which executes the classical e-MDB loop: reading perceptions, calculation of activations (determining relevant contexts), selecting policies, and executing policies.

Here we can define the following:

```yaml
Experiment:
    name: main_loop
    class_name: cognitive_processes.main_loop.MainLoop
    new_executor: True
    threads: 2
    parameters: 
        iterations: 10000
        trials: 20
        subgoals: False
        softmax_selection: True
        softmax_temperature: 0.3
        kill_on_finish: True
```

The relevant parameters that we can change are the number of iterations of the experiment and the number of trials, or policies executed, before resetting the simulated world or the real environment to prevent the experiment from getting stuck. Additional parameters include the option to use a probabilistic selection of policies and killing the architecture processes once the iterations have been reached (Useful for batch executions). 

**Control channel:**

If we need a control channel that communicates the cognitive architecture with another module, such as a simulator or real robot, we can configure that:

```yaml
Control:
    id: ltm_simulator
    control_topic: /main_loop/control
    control_msg: cognitive_processes_interfaces.msg.ControlMsg
    episodes_topic: /main_loop/episodes
    episodes_msg: cognitive_processes_interfaces.msg.Episode
    executed_policy_service: /emdb/simulator/executed_policy
    executed_policy_msg: cognitive_node_interfaces.srv.Policy
    world_reset_service: /emdb/simulator/world_reset
    world_reset_msg: cognitive_processes_interfaces.srv.WorldReset
```

Here we can configure the interfaces names and message types to communicate with the architecture. This includes the current iteration of the experiment, the episodes generated and the services for policy execution and world resets.

**LTM elements:**

We can decide the elements that the cognitive architecture will create when it's launched or during execution:

- **Files:**

    We can decide the output files that the architecture will generate and in which we can check the results of the experiment.

```yaml
LTM:
    Files:
        -
            id: goodness
            class: core.file.FileGoodness
            file: goodness.txt
        -
            id: pnodes_success
            class: core.file.FilePNodesSuccess
            file: pnodes_success.txt
```

    By default, we use the **pnodes_success.txt**, in which we can see the points and anti-points added in each P-Node, and the **goodness.txt**, in which we can see the policy executed in one iteration, the reward obtained, if there have been sensory changes, and more. 

    More files could be configured in the *file.py* script, see the [API reference](https://docs.pillar-robots.eu/projects/emdb_core/en/latest/core/as_API.html#file) for full details.

- **Cognitive Nodes:**
    
    We can add Cognitive Nodes that will be in the LTM from the beginning of the experiment. We have to specify the node type, its name, the class to use, and its parameters:

```yaml
LTM:
    Nodes:
        Perception:
            -
                name: ball_in_left_hand
                class_name: cognitive_nodes.perception.DiscreteEventSimulatorPerception
                parameters:
                    default_msg: std_msgs.msg.Bool
                    default_topic: /mdb/baxter/sensor/ball_in_left_hand
            -
                name: ball_in_right_hand
                class_name: cognitive_nodes.perception.DiscreteEventSimulatorPerception
                parameters:
                    default_msg: std_msgs.msg.Bool
                    default_topic: /mdb/baxter/sensor/ball_in_right_hand
            -
                name: boxes
                class_name: cognitive_nodes.perception.DiscreteEventSimulatorPerception
                parameters:
                    default_msg: simulators_interfaces.msg.ObjectListMsg
                    default_topic: /mdb/baxter/sensor/boxes
                    normalize_data:
                        distance_min: 0.2
                        distance_max: 1.9
                        angle_min: -1.4
                        angle_max: 1.4
                        diameter_min: 0.0
                        diameter_max: 0.15
            -
                name: cylinders
                class_name: cognitive_nodes.perception.DiscreteEventSimulatorPerception
                parameters:
                    default_msg: simulators_interfaces.msg.ObjectListMsg
                    default_topic: /mdb/baxter/sensor/cylinders
                    normalize_data:
                        distance_min: 0.2
                        distance_max: 1.9
                        angle_min: -1.4
                        angle_max: 1.4
                        diameter_min: 0.0
                        diameter_max: 0.15
        WorldModel:
            -
                name: GRIPPER_AND_LOW_FRICTION
                class_name: cognitive_nodes.world_model.WorldModel
        Need: 
            -
                name: object_in_box_need
                class_name: cognitive_nodes.need.Need
                parameters:
                    weight: 1.0
                    drive_id: 'object_in_box_drive'
                    need_type: 'Operational'
            - 
                name: novelty_need
                class_name: cognitive_nodes.need.Need
                parameters:
                    weight: 0.2
                    drive_id: 'novelty_drive'
                    need_type: 'Cognitive'
        Drive:
            -
                name: object_in_box_drive
                class_name: cognitive_nodes.drive.DriveExponential
                parameters:
                    input_topic: /mdb/baxter/sensor/progress
                    input_msg: std_msgs.msg.Float32
                    min_eval: 0.8 
                    neighbors: [{"name": "object_in_box_need", "node_type": "Need"}]

            -
                name: novelty_drive
                class_name: cognitive_nodes.novelty.DriveNovelty
                parameters:
                    neighbors: [{"name": "novelty_need", "node_type": "Need"}]
``` 

- **Connectors:**

The main loop cognitive process can create Cognitive Nodes during execution. To do that, it has to know which class it has to use to create each one. That can be configured in this file:

```yaml
LTM:
    Connectors:
        -
            data: Space
            default_class: cognitive_nodes.space.SVMSpace
        -
            data: Perception
            default_class: cognitive_nodes.perception.Perception
        -
            data: PNode
            default_class: cognitive_nodes.pnode.PNode
        -
            data: CNode
            default_class: cognitive_nodes.cnode.CNode
        -
            data: Goal
            default_class: cognitive_nodes.goal.Goal
        -
            data: WorldModel
            default_class: cognitive_nodes.world_model.WorldModel
        -
            data: Policy
            default_class: cognitive_nodes.policy.Policy
```

- **Others:**

We can add all the parameters that will be needed in eur experimentA

```yaml
SimulatedBaxter:
    Perceptions:
        -
            name: cylinders
            perception_topic: /mdb/baxter/sensor/cylinders
            perception_msg: simulators_interfaces.msg.ObjectListMsg
        -
            name: boxes
            perception_topic: /mdb/baxter/sensor/boxes
            perception_msg: simulators_interfaces.msg.ObjectListMsg
        -
            name: ball_in_left_hand
            perception_topic: /mdb/baxter/sensor/ball_in_left_hand
            perception_msg: std_msgs.msg.Bool
        -
            name: ball_in_right_hand
            perception_topic: /mdb/baxter/sensor/ball_in_right_hand
            perception_msg: std_msgs.msg.Bool
        -
            name: progress_ball_in_box
            perception_topic: /mdb/baxter/sensor/progress
            perception_msg: std_msgs.msg.Float32
```

You can check complete YAML files in the */experiments* directory of the *experiments* package in the [emdb_experiments](https://github.com/pillar-robots/emdb_experiments_gii) repository.


## Launch file

This file launches the processes needed for the operation of the cognitive architecture and defines their input parameters.

The launch file is divided into two Python functions:

**Launch setup:**

Here we have to put the input arguments and parameters that the execution nodes will take. Also, we have to put the essential execution nodes to launch the architecture (commander, ltm) and the other ones that launch our specific modules, such as the Discrete Event Simulator, for example.

```python
def launch_setup(context: LaunchContext, *args, **kwargs):

    logger = LaunchConfiguration("log_level")
    random_seed = LaunchConfiguration("random_seed")
    experiment_file = LaunchConfiguration("experiment_file")
    experiment_package = LaunchConfiguration("experiment_package")
    config_package = LaunchConfiguration("config_package")
    config_file = LaunchConfiguration("config_file")

    core_node = Node(
        package="core",
        executable="commander",
        output="screen",
        arguments=["--ros-args", "--log-level", logger],
        parameters=[{"random_seed": random_seed}],
    )

    ltm_node = Node(
        package="core",
        executable="ltm",
        output="screen",
        arguments=["0", "--ros-args", "--log-level", logger],
    )

    simulator_node = Node(
        package="simulators",
        executable="fruit_shop_simulator",
        output="screen",
        parameters=[
            {
                "random_seed": random_seed,
                "config_file": PathJoinSubstitution(
                    [FindPackageShare(experiment_package), "experiments", experiment_file]
                ),
            }
        ],
    )

    config_service_call = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " ",
                "service call",
                " ",
                "commander/load_config",
                " ",
                "core_interfaces/srv/LoadConfig",
                " ",
                '"{file:',
                " ",
                PathJoinSubstitution(
                    [FindPackageShare(config_package), "config", config_file]
                ),
                '}"',
            ]
        ],
        shell=True,
    )

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=core_node,
            on_exit=[Shutdown()],  
        )
    )

    nodes_to_start = [config_service_call, core_node, ltm_node, simulator_node, shutdown_on_exit]

    return nodes_to_start
```

**Generate launch description:**

Here we have to declare the input arguments and parameters, indicating their name, which has to be the same as the put in the function above; their default value, and their description.

```python
def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "random_seed",
            default_value="0",
            description="The seed to the random numbers generator",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "experiment_file",
            default_value="fruit_shop_experiment.yaml",
            description="The file that loads the experiment config",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "config_file",
            default_value="commander.yaml",
            description="The file that loads the commander config",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "config_package",
            default_value="core",
            description="Package where the config file is located",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "experiment_package",
            default_value="experiments",
            description="Package where the experiment file is located",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
```

Remember that we can change the default values on the command line when we launch our experiment. For example:

```bash
ros2 launch experiments example_launch.py random_seed:=10 log_level:=debug
```

```{warning}
The name of the launch file has to finish in *launch* (i.e example_launch.py) to be found when the ROS workspace is compiled.
```

Obviously, other configurations can be changed and used in the launch files to get the desired behavior, so you can check the [ROS 2 launch documentation](https://docs.ros.org/en/humble/Concepts/Basic/About-Launch.html).

You can check complete launch files in the */launch* directory of the *experiments* package in the [emdb_experiments](https://github.com/pillar-robots/emdb_experiments_gii) repository.
