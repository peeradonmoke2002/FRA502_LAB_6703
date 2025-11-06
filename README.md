# FRA502-LAB-6703
Peeradon Ruengkaew 6703 (Moke)

## Table of Contents


## System Architecture


## Insallation


## Usage



URDF Tree:
world
└── joint_world (fixed)
    └── link_0
        └── joint_1 (revolute)
            └── link_1
                └── joint_2 (revolute)
                    └── link_2
                        └── joint_3 (revolute)
                            └── link_3
                                └── joint_end_effector (fixed)
                                    └── end_effector

Simplified URDF Tree (without irrelevant joints for the kinematics):
world
└── link_1
    └── link_2
        └── link_3
            └── end_effector

DH Parameters: (csv)
,joint,parent,child,d,theta,r,alpha
0,joint_world,world,link_0,0.0,90.0,0.0,90.0
1,joint_1,link_0,link_1,0.0,-0.0,0.0,-90.0
2,joint_2,link_1,link_2,0.2,90.0,0.0,90.0
3,joint_3,link_2,link_3,-0.02,90.0,0.25,0.0
4,joint_end_effector,link_3,end_effector,-0.0,-89.99992,0.28,-90.00008


DH Parameters: (markdown)
|    | joint              | parent   | child        |     d |    theta |    r |    alpha |
|---:|:-------------------|:---------|:-------------|------:|---------:|-----:|---------:|
|  0 | joint_world        | world    | link_0       |  0    |  90      | 0    |  90      |
|  1 | joint_1            | link_0   | link_1       |  0    |  -0      | 0    | -90      |
|  2 | joint_2            | link_1   | link_2       |  0.2  |  90      | 0    |  90      |
|  3 | joint_3            | link_2   | link_3       | -0.02 |  90      | 0.25 |   0      |
|  4 | joint_end_effector | link_3   | end_effector | -0    | -89.9999 | 0.28 | -90.0001 |
