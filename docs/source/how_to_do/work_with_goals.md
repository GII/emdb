# How to work with Motivations 
In this section, we are going to explain how to work with the Goal cognitive node in the context of the e-MDB cognitive architecture of the PILLAR Robots project. Here you will be able to check how to configure robot's motivations either as Goals or Need/Drives node and how to interact with them in order to get the expected behavior.

## How to configure a standalone Goal node

If you want to configure the behavior of a Goal node, you can create your own class in the goal.py script.
The basic methods that must be implemented are the following ones:

- **Get_reward:** The method in charge of calculate the reward obtained after an executed policy. It determines if the Goal is reached (or if the policy helped to get closer to it).

- **Calculate_activation:** The method that determines the activation of the Goal node. It may depend on the iteration of the experiment, a cognitive process, and more. We have to remember that the activation of the Goal node affects the activation of its neighbor C-Node and consequently, of its connected Policy.

Other methods can be implemented to get the desired behavior.

For instance, in the Goal of the example experiment with the discrete event simulator, embedded in a class called GoalObjectInBoxStandalone, this is the implementation of the basic methods:

```python
def calculate_activation(self, perception = None):
    """
    Returns the the activation value of the goal

    :param perception: Perception does not influence the activation 
    :type perception: dict
    :return: The activation of the goal
    :rtype: float
    """
    iteration=self.iteration
    if self.end:
        if(iteration % self.period >= self.start) and (
            iteration % self.period <= self.end
        ):
            self.activation = 1.0
        else:
            self.activation = 0.0

    if self.activation_topic:
        self.publish_activation(self.activation)
    return self.activation

async def get_reward(self):
    """
    Calculate the reward for the current sensor values.

    :return: The reward obtained
    :rtype: float
    """
    self.reward = 0.0
    for activation in [self.activation]:
        if (self.sensorial_changes()) and isclose(activation, 1.0):
            if (await self.object_in_close_box()) or (await self.object_in_far_box()):
                self.reward = 1.0
            elif self.object_held():
                if self.object_held_with_two_hands():
                    self.reward = 0.6
                elif self.ball_and_box_on_the_same_side():
                    self.reward = 0.6
                elif not self.object_held_before():
                    self.reward = 0.3
            elif not self.object_held_before():
                if (await self.object_pickable_with_two_hands()):
                    self.reward = 0.3
                elif (await self.object_was_approximated()):
                    self.reward = 0.2
    return self.reward
```

The node activation depends on the iteration of the experiment and the reward can be lower than 1.0, if the executed policy helped to get closer to the Goal, or 1.0 if it led to reach it. As we can see, there are other methods that are necessary to calculate the reward obtained, for example.

<!-- Quizá se debería añadir un apartado del por qué del uso de asyncio -->

## How to interact with a Goal node

The way to interact with the Goal nodes is by using, mainly, three ROS services:
- **Get_reward service:** If we call this service, we'll receive the last reward calculated by the get_reward method.
- **Is_reached service:** Calling this service we'll receive if the Goal was reached in the last iteration of the experiment.
- **Set_activation service:** We can set the activation of the node calling this service, independently of the calculate_activation method.

The way to use this service depends on the specific application. For instance, in the example experiment with the discrete event simulator, it makes no sense to use only the is_reached service, because we have intermediate rewards and they are necessary to create the P-Nodes and C-Nodes of several policies. 

Also, if we want to set the activation of the node manually, without calculating it internally, we can create a dummy calculate_activation method and only modify the value with the set_activation service.

```python
def calculate_activation(self, perception = None):
    """
    Returns the the activation value of the goal

    :param perception: Perception does not influence the activation 
    :type perception: dict
    :return: The activation of the goal
    :rtype: float
    """
    if self.activation_topic:
        self.publish_activation(self.activation)
    return self.activation
```

## Linking needs, drives and goals

In the latest version of the architecture **Need** and **Drive** nodes were implemented. These nodes alter both the activation and reward provided by the goal. A simple setup would include a need, drive and goal linked, as in the following example:

```yaml
LTM:
    Nodes:
        Need: 
            -
                name: object_in_box_need
                class_name: cognitive_nodes.need.Need
                parameters:
                    weight: 1.0
                    drive_id: 'object_in_box_drive'
                    need_type: 'Operational'
        Drive:
            -
                name: object_in_box_drive
                class_name: cognitive_nodes.drive DriveExponential
                parameters:
                    input_topic: /mdb/baxter/sensor/progress
                    input_msg: std_msgs.msg.Float32
                    min_eval: 0.8 
                    neighbors: [{"name": "object_in_box_need", "node_type": "Need"}]
        Goal:
            -
                name: object_in_box_goal
                class_name: cognitive_nodes.goal.GoalMotiven
                parameters:
                    neighbors: [{"name": "object_in_box_drive", "node_type": "Drive"}]
```

This setup allows for full control on the behavior of the different nodes, as each class can be customized as required. Beyond this, the architecture now allows the goals to be automatically discovered and learned. This applies to all drives that do not have a goal linked beforehand. The architecture will create a goal of the class specified in the *connectors* parameter when a reward is detected in a drive. The following example shows how to set up this configuration:

```yaml
LTM:
    Connectors:
        -
            data: Goal
            default_class: cognitive_nodes.goal.GoalLearnedSpace
    Nodes:
        Need: 
            -
                name: object_in_box_need
                class_name: cognitive_nodes.need.Need
                parameters:
                    weight: 1.0
                    drive_id: 'object_in_box_drive'
                    need_type: 'Operational'
        Drive:
            -
                name: object_in_box_drive
                class_name: cognitive_nodes.drive DriveExponential
                parameters:
                    input_topic: /mdb/baxter/sensor/progress
                    input_msg: std_msgs.msg.Float32
                    min_eval: 0.8 
                    neighbors: [{"name": "object_in_box_need", "node_type": "Need"}]
        Goal:
            -
                name: object_in_box_goal
                class_name: cognitive_nodes.goal.GoalMotiven
                parameters:
                    neighbors: [{"name": "object_in_box_drive", "node_type": "Drive"}]
```

### How to interact with a Need node

There are two ROS2 services to interact with the Need nodes:

- **Set_activation service** With this service the activation of the Need node can be changed, which can be used to modify how strongly a need is prioritized.
- **Get_satisfaction service** With this service we can ask whether the need is satisfied or not. This means, that the linked drive has an evaluation of 0. 

### How to interact with a Drive node

Currently, the implementation of the Drive node reads from a topic that has to be specified in the configuration file (class DriveTopicInput), different drive functions can be created by subclassing (e.g class DriveExponential). Once the node is configured, the following ROS2 interfaces can be used to interact with it:

- **evaluation topic** In this topic the current value of the Drive's activation is published.
- **get_reward service** This service is used to ask if the drive has detected a reward. A reward is detected when a reduction in the evaluation value is detected, calling this service will reset the detected reward value, therefore, subsequent calls will result in 0 reward unless a further reduction happens.

## The GoalMotiven and GoalLearnedSpace classes

These classes have been implemented to automatically work with drives to facilitate obtaining rewards. In the case of the GoalLearnedSpace class, a space with the rewarded and not rewarded points is also learned automatically.


