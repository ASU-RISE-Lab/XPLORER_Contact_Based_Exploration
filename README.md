# XPLORER_Contact_Based_Exploration
Repo containing Exploration Trajectory generation code for XPLORER

## Description

You need to have the following packages installed in your system:

[SQUEEZE Custom Messages](https://github.com/ASU-RISE-Lab/squeeze_custom_msgs)

## Build

```
cd ~/colcon_ws/src
git clone git@github.com:ASU-RISE-Lab/XPLORER_Contact_Based_Exploration.git
cd ~/colcon_ws
colcon build --packages-select Exploration_Algorithm
```

## Launching the node

```
cd Exploration_Algorithm/Exploration_Algorithm/
python3 Exploration.py
```
