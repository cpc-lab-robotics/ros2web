# ROS2Web

ROS2Web enables the creation of Web user interfaces by combining user interface components and writing data bindings. It also provides an API that extends the ros2cli and ROS2 launch, making it easy to incorporate ROS2 system functions such as getting a package list and executing packages into their development applications. Its API also can create a Web API by simply adding a specified decorator to a function. 

# Install
```zsh
pip install aiohttp dacite tinydb hashids

mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/cpc-lab-robotics/ros2web.git
colcon build --symlink-install
source /opt/ros/galactic/setup.zsh
. ./install/local_setup.zsh
```

# Usage

```zsh
cd ros2_ws/src
ros2 web create example
cd ..
colcon build --symlink-install
. ./install/local_setup.zsh
ros2 web server
```

After starting up the server, try accessing this URL.

http://localhost:8080/api/example/package_names


The only program to be written for this WebAPI is the following.

```python
@api_def.get("/package_names")
async def package_names(self):
    return await self.ros2.pkg.list()
```

### Examples

- https://github.com/cpc-lab-robotics/ros2web_graph
