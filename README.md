# e-MDB meta repository

This is the meta repository that includes all the packages needed for correct and complete operation of the e-MDB cognitive architecture software.

The architecture, at this moment, is divided into five GitHub repositories:

- [e-MDB core components](https://github.com/pillar-robots/emdb_core) - Core of the cognitive architecture.
- [Cognitive nodes implemented by the GII](https://github.com/pillar-robots/emdb_cognitive_nodes_gii) - Reference implementation for the main cognitive nodes.
- [Cognitive processes implemented by the GII](https://github.com/pillar-robots/emdb_cognitive_processes_gii) - Reference implementation for the main cognitive processes.
- [Discrete event simulator to be used in experiments](https://github.com/pillar-robots/emdb_discrete_event_simulator_gii) - Implementation of discrete event simulators used in experiments.
- [Experiments designed by the GII](https://github.com/pillar-robots/emdb_experiments_gii) - Configuration files for experiments.

You can find information and tutorials about the e-MDB cognitive architecture software in the [official documentation](https://docs.pillar-robots.eu/en/latest/)

# How to install

Refer to our [installation guide](https://docs.pillar-robots.eu/en/latest/how_to_do/how_to_install.html).

# Contribution Guide

In order to update the links to the latest commits of the submodules do the following:

1. Merge changes in each of the packages. 

2. Update the submodules:

```
git submodule update --remote --merge
git add *
git commit -m "Updated submodules to latest branch commit"
git push origin main
```


Grant PID2021-126220OB-I00 funded by MCIN/AEI/ 10.13039/501100011033 and by "ERDF A way of making Europe".

![Grant PID2021-126220OB-I00 funded by MCIN/AEI/ 10.13039/501100011033 and by "ERDF A way of making Europe".](docs/source/images/micin-uefeder-aei.jpg)
