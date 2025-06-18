# How to install the e-MDB cognitive architecture software

To install the software of e-MDB, firstly it's necessary to create a ROS workspace and clone the following GitHub meta repository:

[wp5_gii meta repository](https://github.com/pillar-robots/wp5_gii)

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

```{note}
Remember to install the dependencies!

- ROS 2 Humble
- Numpy 1.24.2
- Sklearn 1.4.2
- Tensorflow 2.16.1
- Yamlloader 1.4.1
- Pandas 2.2.3
- Ollama 0.4.7

*Other versions could work, but the indicated ones have proven to be functional.*
```

