# Optitrack Obstacle Specification

The Optitrack Obstacle Specification project enables the specification of obstacles using the Optitrack system. It collects pose measurements from the Optitrack system and provides functionality to define obstacles by extracting position and orientation data.

## Launching the Optitrack Collector Node

To launch the Optitrack Collector node, follow these steps:

1. Ensure that you have the ROS environment properly set up, including the installation of the necessary dependencies.

2. Clone this repository into your ROS workspace:
```
git clone https://github.com/your-username/hri_optitrack
```

## Setup ROS master to match UR PC
```
export ROS_MASTER_URI:=http://192.168.1.100:11311
export ROS_IP:=192.168.1.50
```

## Questions 

What's the difference between following ROS topics: 
```
/kdno/INST_POINT
/vrpn_client_node/INST_POINT/pose 
```

## TODO: 

- [ ] Add dimensions for the first shape (rectangle) 
- [ ] Add service call for creating obstacle in env 
- [ ] Check necessary transformations for putting stuff in robot coordinate frame 
- [ ] Add specification of the plane only by points 
