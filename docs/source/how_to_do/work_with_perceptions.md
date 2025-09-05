# How to work with Perceptions
In this section, we are going to explain how to work with the Perception cognitive node in the context of the e-MDB cognitive architecture of the PILLAR Robots project. Here you will be able to check how to configure a Perception node or how to interact with it, in order to get its expected behavior.

## How to create a new perception node type

If you want to configure the behavior of a Perception node, you can create your own class by inheriting the base class from the perception.py module.

The basic method that must be implemented is the **process_and_send_reading**, which is in charge of transforming the perceptions received from a simulator or a real robot to the format used in the cognitive architecture. Then, it publishes the transformed perception in the Value topic. The format of the perception is the following one:

```python
{'sensor_name':[sensor_value0, sensor_value1, ...]}

# The sensor values must be dictionaries. i.e: sensor_value0 = {'distance':10}
```

For instance, in the case of the example experiment with the discrete event simulator, the method is implemented in this way:

```python
def process_and_send_reading(self):
    """
    Method that processes the sensor values received
    """
    sensor = {}
    value = []
    if isinstance(self.reading.data, list):
        for perception in self.reading.data:
            distance = (
                perception.distance - self.normalize_values["distance_min"]
            ) / (
                self.normalize_values["distance_max"]
                - self.normalize_values["distance_min"]
            )
            angle = (perception.angle - self.normalize_values["angle_min"]) / (
                self.normalize_values["angle_max"]
                - self.normalize_values["angle_min"]
            )
            diameter = (
                perception.diameter - self.normalize_values["diameter_min"]
            ) / (
                self.normalize_values["diameter_max"]
                - self.normalize_values["diameter_min"]
            )
            value.append(
                dict(
                    distance=distance,
                    angle=angle,
                    diameter=diameter,
                    # id=perception.id,
                )
            )
    else:
        value.append(dict(data=self.reading.data))

    sensor[self.name] = value
    self.get_logger().debug("Publishing normalized " + self.name + " = " + str(sensor))
    sensor_msg = perception_dict_to_msg(sensor)
    self.perception_publisher.publish(sensor_msg)
```

In this experiment, we have four different Perception nodes, one per sensor:

- **Cylinders**: This sensor detects the distance, angle, and diameter of the cylinders on the simulated table.
- **Boxes**: This sensor detects the distance, angle, and diameter of the boxes on the simulated table.
- **Ball_in_right_hand**: This sensor detects if the simulated robot has a cylinder in its right hand. 
- **Ball_in_left_hand**: This sensor detects if the simulated robot has a cylinder in its left hand. 

With that implementation, the transformed perception of these sensors has the following format:

```python 
{'cylinders': [{'distance': 0.18597018918340155, 'angle': 0.503317891044131, 'diameter': 0.46666666666666673}]}

{'boxes': [{'distance': 0.2744255090024155, 'angle': 0.1793823313667857, 'diameter': 0.8}]}

{'ball_in_right_hand': [{'data': True}]}

{'ball_in_left_hand': [{'data': False}]}

# In this case, there is only one sensor_value in each perception
```

In conclusion, we can add to the architecture the perceptions we want, but we need to create a process_and_send_reading that transforms it to the format indicated above.

It's important to remember that, for correct behavior of the cognitive architecture, the perception values **must be normalized.** That normalization process can be done in the process_and_send_reading method, indicating the values to normalize in the experiment configuration YAML file.

## How to configure a Perception node

The created Perception node classes can be added to the architecture using the YAML configuration file of the experiment. All parameters included in the constructor can be set in the *parameters* field. We have a default topic configured in the configuration YAML file of the experiment from which the Perception node reads the original sensor values, coming from the robot or simulator, before transforming them.

```yaml
    Nodes:
        Perception:
            -
                name: button_light
                class_name: cognitive_nodes.perception.FruitShopPerception
                parameters:
                    default_msg: std_msgs.msg.Bool
                    default_topic: /emdb/simulator/sensor/button_light
```


## How to interact with a Perception node

The way to interact with a perceptual node, mainly, is by calling the **set_activation service** or by subscribing to the **value topic**.

<!-- Hay que implementarlo en el main loop!! -->
<!-- In the case of the activation, the default value of the Perception node is 1.0, and it's implemented by a dummy calculate_activation method, that only returns the current activation value. Because of that, we can decide if a Perception node works or not during execution by calling the set_activation service. If we change one activation to 0.0, the information of that node won't enter the architecture. -->

In the Value topic, we can read the transformed and normalized value of the perception and we can use it in the cognitive architecture.

