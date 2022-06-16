# Navigation Goal Queue

Allows setting multiple navigation goals which are executed sequentially.

## Subscribed Topics
```   
~/goal [geometry_msgs::PoseStamped]
    used instead move_base_simple/goal
    
~/reset [geometry_msgs::PointStamped]
    clear goal queue (point not used)
```

## Published Topics
```
move_base/goal [move_base_msgs::MoveBaseActionGoal]
    current goal
    
~/path [nav_msgs::Path]
    path for visualization
```

## Parameters
```
loop [bool, default: true]
    continue from the first goal again when finished
```
