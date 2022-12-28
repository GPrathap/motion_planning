# waypoint_navigation_plugin

For testing
```
roslaunch waypoint_navigation_plugin rviz.launch
```

Add the WaypointNav Tool from waypoint_navigation_plugin

![WP1](doc/wp_doc_001.png "WP1")

Click on the tool to add multiple waypoints and drop onto RviZ scene. The locations can be updated by dragging the Interactive marker of by using the Rviz panel
![WP2](doc/wp_doc_002.png "WP2")

Clicking "Publish Waypoints" on the panel publishes nav_msgs::Path on topic entered under "Topic"
![WP3](doc/wp_doc_003.png "WP3")