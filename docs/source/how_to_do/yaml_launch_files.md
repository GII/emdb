# How to configure an experiment

In this section, we are going to see how to use the YAML and launch files in order to configure our experiments.

These files appear in other How To Do sections, but here we will do a complete explanation of both.

## YAML file

The YAML file defines the parameters of the experiment that determine the behavior of the cognitive architecture.
We can configure the following ones:

**Cognitive processes:**

We can select the cognitive processes to execute and define their parameters. 

Currently, we only have one cognitive process: the main loop, which executes the classical e-MDB loop: reading perceptions, calculation of activations (determining relevant contexts), selecting policies, and executing policies.

Here we can define the following:

```yaml
Experiment:
    name: main_loop
    class_name: cognitive_processes.main_loop.MainLoop
    new_executor: False
    threads: 2
    parameters: 
        iterations: 6000
        trials: 50
        subgoals: False
```

The relevant parameters that we can change are the number of iterations of the experiment and the number of trials, or policies executed, before resetting the simulated world or the real environment to prevent the experiment from getting stuck.

**Control channel:**

If we need a control channel that communicates the cognitive architecture with another module, such as a simulator or real robot, we can configure that:

```yaml
Control:
    id: ltm_simulator
    control_topic: /main_loop/control
    control_msg: core_interfaces.msg.ControlMsg
    executed_policy_topic: /mdb/baxter/executed_policy
    executed_policy_msg: std_msgs.msg.String
```

Here we can configure the topic in which control actions are published  (*/main_loop/control/*) and its message (*core_interfaces.msg.ControlMsg*) or the topic in which the policy to execute is published (*/mdb/baxter/executed_policy*) and its message (*std_msgs.msg.String*).

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

    More files could be configured in the *file.py* script.

- **Cognitive Nodes:**
    
    We can add Cognitive Nodes that will be in the LTM from the beginning of the experiment. We have to specify the node type, its name, the class to use, and its parameters:

    ```yaml
    LTM:
        Nodes:
            Perception:
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
            WorldModel:
                -
                    name: GRIPPER_AND_LOW_FRICTION
                    class_name: cognitive_nodes.world_model.WorldModel
            Goal:
                -
                    name: object_in_box_standalone
                    class_name: cognitive_nodes.goal.GoalObjectInBoxStandalone
                    parameters:
                        normalize_data:
                            distance_min: 0.2
                            distance_max: 1.9
                            angle_min: -1.4
                            angle_max: 1.4
                            diameter_min: 0.0
                            diameter_max: 0.15
                        data:
                            space: cognitive_nodes.space.NormalCentroidPointBasedSpace
                            points:
                                -
                                    cylinders:
                                        dist: 0.575
                                        angle: 0.0
                                    boxes:
                                        dist: 0.575
                                        angle: 0.0
                                    ball_in_left_hand:
                                        data: False
                                    ball_in_right_hand:
                                        data: False
                            start: 0
                            end: 3000
                            period: 3000
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
```

You can check complete YAML files in the */experiments* directory of the *experiments* package in the [emdb_experiments](https://github.com/pillar-robots/emdb_experiments_gii) repository.


## Launch file

This file launches the processes needed for the operation of the cognitive architecture and defines their input parameters.

The launch file is divided into two Python functions:

**Launch setup:**

Here we have to put the input arguments and parameters that the execution nodes will take. Also, we have to put the essential execution nodes to launch de architecture (commander, ltm) and the other ones that launch our specific modules, such as the Discrete Event Simulator, for example.

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
        executable="simulator",
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
            default_value="default_experiment.yaml",
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
