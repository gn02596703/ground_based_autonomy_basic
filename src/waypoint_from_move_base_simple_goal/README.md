## Waypoint creation from move_base_simple/goal

The node lets the user to set target waypoints for ground_autonomy system.<br>
It is modified version of waypointExample node.

In addition to set waypoint list from reading ply file, it can create new waypoint from ```/move_base_simple/goal``` topic which you can set by rviz ```2D Nav Goal``` button and pubishes the waypoint as ```/way_point``` topic. 

To-do:
-   create logic to handle waypoint that can not be reached