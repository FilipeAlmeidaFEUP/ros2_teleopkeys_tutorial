# Flatland Teleopkeys Tutorial using ROS 2

This tutorial will show you how it is possible to use ROS 2 and the Flatland Simulator to create a simulated world with a robot and how to control it using the keyboard. It also briefly explains how these tools function but, if you wish to learn more about them, please visit the [ROS 2 Documentation](https://docs.ros.org/en/humble/) and the [Flatland Documentation](https://flatland-simulator.readthedocs.io/en/latest/). All the code in this project is written in python.

# Setup and Pre-requisites

## ROS 2 Humble Instalation

This tutorial was built to work with ROS 2 Humble which is, at the time of writing, the latest distribution. Follow the instructions in the official [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html). 
It is recommended to at least install the desktop version, as some tools that will be used here are already contained within.

## Flatland 2

A fork of Flatland modified to work on ROS 2 natively is available on [this repository](https://github.com/JoaoCostaIFG/flatland). Follow the instalation instructions to have this version available on your machine.

## Workspace setup

To be able to build all the dependencies and run your projects, ROS needs to a designated folder known as the workspace. To setup your workspace follow the oficial [ROS 2 tutorial on how to create a workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

# Clone the repository

After everything is setup, run the following commands inside your ROS 2 workspace folder to clone this repository as a package:

```
cd src/
git clone https://github.com/FilipeAlmeidaFEUP/ros2_teleopkeys_tutorial.git
```

# Building and installing dependencies

Before running the package for the first time, you need to execute all these commands:

1. Resolve dependencies:`rosdep install -i --from-path src --rosdistro humble -y`
2. Build the workspace:`colcon build`
3. Source the setup script:`source install/setup.bash`

After the first run, these commands might be usefull for this or any other package in your workspace:

- For every new terminal that you open that accesses the workspace, run command 3.
- If you made changes in any file of any package, you need to either build the entire workspace with command 2 or use the same command with [arguments](https://colcon.readthedocs.io/en/released/reference/package-selection-arguments.html) to build only specific packages.
- If there are new dependencies on any of your packages (ex.:new python imports), you should run command 1 to make sure all of them are resolved.
- If you creating a new package or cloning one, run all the commands.

# Running the package

Once everithing is built, running the package can be done with the command:
```
ros2 launch serp_teleop serp_teleop.launch.py
```

If everything worked as intended, you should be seeing a window that looks like this:

![Flatland Window](images/flatland_window.png)

Right now you are seeing the robot move inside the map built for this package but, although the space is not being used, the world extends beyond the walls on the screen. To navigate through the flatland world use these mouse controls:

- Scroll button + drag: move the window.
- Scroll up/down: zoom in/out.
- Left click + drag: rotate the window.

The window on the left called Displays allows you to control some aspects of the visualization. For example, try to unselect the checkbox called `LaserScan (kinect)`. Once you do, the red squares around the robot will disappear. These squares were representing the the collisions from the robot's radar with the walls. Turning this off does not mean the radar is no longer working, only that it is not appearing in the visualization.

For now, the robot can not be controlled with the keyboard. It moves forward until detects a wall in front of it using its radar. When a wall is detected, it randomly chooses one direction to rotate 90º, essentially following a random path. The next section will briefly explain how a Flatland ROS 2 package is structured. If you wish to skip this part and go directly to the keyboard control, go to [this section](#keyboard-control).

# The Flatland ROS 2 package

## ROS 2

To help you understand the structure of ROS, take a quick look in the documentation sections [Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html), [Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) and [Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html). You don't need to perform any of the tasks, just read first few sections in each and watch the animations that are provided.

### Launch file

ROS 2 provides the `run` run command that starts one node but, in some cases, like in this package, you might want to start several nodes at once. To do that you can use a launch file such as the `launch/serp_teleop.launch.py` in this repository. For more information on this type of file, consult the documentation on [creating launch files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html). A total of 4 nodes are launched by this file but, for now, lets focus on only 2:

- The `flatland_server` node: runs the Flatland simulation, including the robot and the world.

```
ld = LaunchDescription(
    [
        DeclareLaunchArgument(name="world_path", default_value=PathJoinSubstitution([pkg_share, "world/world.yaml"])),
        DeclareLaunchArgument(name="update_rate", default_value="100.0"),
        DeclareLaunchArgument(name="step_size", default_value="0.01"),
        DeclareLaunchArgument(name="show_viz", default_value="true"),
        DeclareLaunchArgument(name="viz_pub_rate", default_value="30.0"),
        DeclareLaunchArgument(name="use_sim_time", default_value="true"),

        [...]

        # launch flatland server
        Node(
            name="flatland_server",
            package="flatland_server",
            executable="flatland_server",
            output="screen",
            parameters=[
                # Use the arguments passed into the launchfile for this node
                {"world_path": world_path},
                {"update_rate": update_rate},
                {"step_size": step_size},
                {"show_viz": show_viz},
                {"viz_pub_rate": viz_pub_rate},
                {"use_sim_time": use_sim_time},
            ],
        ),
        [...]
    ]
)
```
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
This node has several arguments that modify the functioning of the simulator. For a more detailed explanation on these parameters go to Flatland documentation on [how to launch Flatland server node](https://flatland-simulator.readthedocs.io/en/latest/core_functions/ros_launch.html#).

- The `serp_teleop` node: runs the code in the file `serp_teleop/__init__.py` that is used to control the robot.

```
ld = LaunchDescription(
    [
        [...]
        Node(
            name="serp_teleop",
            package="serp_teleop",
            executable="serp_teleop",
            output="screen",
        ),
        [...]
    ]
)
```

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
This node needs to have an instance of the class `rclpy.node.Node` created and initialized. This can be done by:

```
node = rclpy.create_node('node_name')
```

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
In this package this is achieved by creating a class that inherits from the `rclpy.node.Node` class and initializing it:

```
class NewNode(Node):
    def __init__(self) -> None:
        super().__init__("node_name")
```

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
Both methods are equally valid but the second one might be better to keep the code organized in more complex projects. For any other code examples in this tutorial, consider the variable `node` as an initialized instance of the `rclpy.node.Node` class.

### Using Topics and Services

The `flatland_server` node publishes to several topics the `serp_teleop` node needs to subscribe to. To do so, the following code is necessary for each subscription:

```
#create a subscription
node.create_subscription(Msg_Type, "/topic_name", handling_function, queue_size)

#each time a message is published, this function is executed and the arg data is the msg 
def handling_function(data):
    [...]
```


The `flatland_server` also subscribes to some topics the `serp_teleop` node will publish to. The following code shows how a message can be subscribed to a topic:

```
#create a publisher
node.pub:Publisher = self.create_publisher(Msg_Type, "/topic_name", queue_size)

#call this function to send a message to the topic
msg = Msg_Type()
publisher.publish(msg)
```

The `serp_teleop` node also needs to use [Flatland services](https://flatland-simulator.readthedocs.io/en/latest/core_functions/ros_services.html#). This code sends a request to a service:

```
client = node.create_client(Msg_Type, "/service_name")
client.wait_for_service()
request = Msg_Type()
client.call_async(request)
```

NOTE: Msg_Type is only a placeholder for these examples and each topic has a different format to their messages. For each case look for the proper documentation to help you.

### Other setup files

Although most setup files don't require a lot of attention and are very simple, there are some canfigurations that you need to check in your packages.

One of them is to chech that you have all the dependencies in the `package.xml` file. For example, in this project:
 ```
<exec_depend>rclpy</exec_depend>
<exec_depend>flatland_msgs</exec_depend>
 ```

You also need to pay some attention to the `setup.py` file. Check that all the files that need to be installed are in the `data_files` list:

```
setup(
    [...]
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch/", glob("launch/*launch*")),
        ('share/' + package_name + "/rviz/", glob("rviz/*")),
        ('share/' + package_name + "/world/" , glob('world/*')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    [...]
)
```

Also make sure you have a proper entry point for you node to start running:

```
setup(
    [...]
    entry_points={
        'console_scripts': [
            'serp_teleop = serp_teleop.__init__:main'
        ],
    },
)
```

For more information on the `setup.py` file go to the [guide on how to develop a ROS 2 python package](https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html#python-packages).

# Keyboard control