# OnRobot two-fingered gripper

This package contains the URDF files describing OnRobot two-fingered grippers (RG2 and RG6).
Use the following to see the model in rviz:

```
ros2 launch onrobot_rg_control bringup.launch.py
```

## Visual and Collision models
### RG2
<img src="images/rg2_visual.png" height="200">  <img src="images/rg2_collision.png" height="200">  

### RG6
<img src="images/rg6_visual.png" height="200">  <img src="images/rg6_collision.png" height="200">  

## Reference
- To generate a collision model, you can use [rosmodelgen](https://github.com/takuya-ki/rosmodelgen)
