# ros2-ws-2024
General working ROS workshop with all packages.


In root directory make sure to run:

```
rosdep install -i --from-path src --rosdistro iron -y
colcon build --packages-select <your_package>
source install/setup.bash
```

You can just run colcon build by itself to build everything but it might take a bit.
Then run your package:

```
ros2 run <package> <entrypoint>
```

Ex.) `ros2 run py_pubsub listener`
