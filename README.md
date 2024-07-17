# e-MDB meta repository

This is the meta repository that includes all the packages needed for correct and complete operation of the e-MDB cognitive architecture software.

The architecture, at this moment, is divided into five GitHub repositories:

- [e-MDB core components](https://github.com/pillar-robots/emdb_core) - Core of the cognitive architecture.
- [Cognitive nodes implemented by the GII](https://github.com/pillar-robots/emdb_cognitive_nodes_gii) - Reference implementation for the main cognitive nodes.
- [Cognitive processes implemented by the GII](https://github.com/pillar-robots/emdb_cognitive_processes_gii) - Reference implementation for the main cognitive processes.
- [Discrete event simulator to be used in experiments](https://github.com/pillar-robots/emdb_discrete_event_simulator_gii) - Implementation of a discrete event simulator used in many experiments.
- [Experiments designed by the GII](https://github.com/pillar-robots/emdb_experiments_gii) - Configuration files for experiments.

You can find information and tutorials about the e-MDB cognitive architecture software in the [PILLAR Robots official documentation](https://docs.pillar-robots.eu/en/latest/)

# How to install

To install the software of e-MDB, firstly it's necessary to install the needed dependencies:

- ROS 2 Humble
- Numpy 1.24.3
- Sklearn 1.4.2
- Tensorflow 2.15.0
- Yamlloader 1.3.2

(*Other versions could work, but the indicated ones have proven to be functional.*)

The next step is to create a ROS workspace and clone this meta repository:

```bash
mkdir -p ~/eMDB_ws/src

cd ~/eMDB_ws/src

git clone --recursive https://github.com/pillar-robots/wp5_gii.git
```

Finally, you have to compile the ROS workspace:

```bash
cd ~/eMDB_ws

colcon build --symlink-install
```
