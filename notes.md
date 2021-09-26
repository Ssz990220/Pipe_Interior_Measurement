# Notes for useful tools

* [Trajectory execution]:https://ww2.mathworks.cn/help/robotics/ug/plan-and-execute-collision-free-trajectory-kinova-gen3.html	"Trajectory execution"

  Closed-Loop Trajectory Planning, Execute Planned Trajectory using Joint-Space Robot Simulation and Control

* Collision Detection: `openExample('robotics/CheckForEnvironmentalCollisionsWithManipulatorsExample')`

* Great Visualization

`openExample('robotics/CheckForEnvironmentalCollisionsWithManipulatorsExample')`

* `plannerBiRRT` for Bidirectional RRT

* Non-convex mesh to Convex meshes conversion: 

  [link]:https://github.com/kmammou/v-hacd

* Replace a collision body

```matlab
clearCollision(franka.Bodies{9});
addCollision(franka.Bodies{9}, "cylinder", [0.07, 0.05], trvec2tform([0.0, 0, 0.025]));
```

* Manipulator RRT

[Manipulator RRT]: https://ww2.mathworks.cn/help/robotics/ug/pick-and-place-using-rrt-for-manipulators.html?searchHighlight=RRT&amp;s_tid=srchtitle
