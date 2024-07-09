# How to work with Cognitive Nodes
In this section, we are going to explain the basics to work with Cognitive Nodes. These are the first steps that are necessary to be read in order to understand the other more specific tutorials that allow you to get the knowledge to add your own implementations to the cognitive architecture.

## How to add a Cognitive Node to the architecture

A Cognitive Node can be added in the experiment YAML file, in order to be part of the architecture from the beginning of the experiment, or can be added afterwards, during the execution.

In the first case, we have to create a YAML file in the experiments package. We can use as a guide the files of the example experiments in the [emdb_experiments_gii](https://github.com/pillar-robots/emdb_experiments_gii) repository. In this file, we have to add our node in the Nodes section, indicating its type, its name, the class that it will use, and the parameters needed for its operation, that must be contemplated in the constructor of the used class.

Here we can see an example in which Perception nodes are added:

```yaml
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
                    diameter_max: 0.15U
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
```

For more information about Perception nodes, you can check this [dedicated section](../how_to_do/work_with_perceptions.md).

In the second case, we have to use the services dedicated to add and delete nodes during execution, which are the following ones:

- **CreateNode service:** You can add a node calling this service and indicating its name, the class that it will use and the parameters needed for its operation
- **DeleteNode service:** You can delete a node from the cognitive architecture calling this service and indicating its name. 

At this moment, the behavior of the cognitive architecture is determined by the cognitive process called main loop, which executes the classical e-MDB loop: reading perceptions, calculation of activations (determining relevant contexts), selecting policies, and executing policies. In this process, it has to create new nodes, calling the CreateNode service. For that, it needs to know the class that the new node has to use, which is indicated in the experiment YAML file, in the Connectors section:

```yaml
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

The Space class is used in the PNode to store the points and anti-points and to calculate its activation. It can be used in Goals too.

## How to interact with Congnitive Nodes

There are some common topics and services with which we are able to interact with the cognitive nodes present in the architecture:

- **Get_activation service:** There are two ways to use this service: sending a perception, with the format specified in [this section](../how_to_do/work_with_perceptions.md), or sending an empty dictionary ({}). In the first case, the node will calculate its new activation with the received perception and will return it. In the second case, the node will return its current activation. There are nodes in which the perception doesn't affect to the activation, so it will return its current activation too, although we send a perception.

- **Get_information service:** This service returns information about a specific node, such as its name, its node type, its current activation, or the name and type of its neighbor nodes.

- **Activation topic**: At this moment, if the activation topic is activated, the node publishes its activation every time its calculate_activation method is called.

- **Set_activation_topic service**: You can activate or deactivate the Activation topic by calling this service.

In addition to these, each cognitive node can have its own topics and services.

## Cognitive Nodes neighbors

To get the correct flow of the activations into the cognitive architecture and get its correct behavior, it's essential to define correctly the neighbors of each cognitive node. The main loop cognitive process is in charge of doing this, but we can do it manually by calling the following services:

- **Add_neighbor service:** We can add a neighbor to a cognitive node by calling this service and indicating the name and node type of the neighbor.
- **Delete_neighbor service:** We can delete a neighbor of a cognitive node by calling this service and indicating the name and node type of the neighbor.

It's important to remember that there is one add_neighbor and delete_neighbor service per cognitive node.

