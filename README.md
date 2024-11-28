# Mowito_task


- Clone the repository

```bash
https://github.com/kiwikev23/mowito_task
```

- Package contains 2 scripts, image_converter --> server node, mode_selector --> client node and a launch file --> nodes.launch.py

- Colcon build the package after cloning the package into your workspace. 
 ```bash
colcon build --packages-select mowito_task
 
```
- Run the launch file to launch the nodes
 ```bash
ros2 launch mowito_task nodes.launch.py 
```
- (Optional) Run the client node to test the service calls
 ```bash
ros2 run mowito_task mode_selector
```
