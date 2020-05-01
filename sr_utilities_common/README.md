# sr_utilities_common
This package contains resources commonly used in another Shadow Robot's packages and repositories

## Shutdown handler
A custom handler for SIGINT signal (ctrl + C) executing a chosen class method apon node shutdown. It's advantage over [rospy.on_shutdown](http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown#Registering_shutdown_hooks) is that it allows using publishers at the shutdown stage. 

Example usage:
```python
class ExampleClass():
    self.__init__():
        self.example_variable

    def example_loop_method():
        while not rospy.is_shutdown():
            rospy.loginfo("Doing some stuff!")

    def clean_up():
        rospy.loginfo("Cleaning up!")

if __name__ == '__main__':
    rospy.init_node('example_node', anonymous=True)

    example_class = ExampleClass()
    shutdown_handler = ShutdownHandler(example_class, 'clean_up()')
    example_class.example_loop_method()

```

The above code will be executing an infinite loop until shutdown for the node is initiated. Then, before the node dies, the `clean_up` method will be executed. 

## Real time tf republisher
This tool allows to republish tfs from a rosbag in real time, i.e. each tf from bag file will be intercepted and republished with current time instead of it's bagging time. Usage:

```sh
roslaunch sr_utilities_common real_time_tf_republisher.launch rosbag_path:=<rosbag_path> remapped_tf_topic_name:=<remapped_tf_topic_name>
```
where:
- `<rosbag_path>` - path to the rosbag you want to replay tf topic from (required)
- `<remapped_tf_topic_name>` - topic name to which `tf` topic from the bag will be remapped to (default: `tf_bag`)

## Ros Heartbeat
A generic ros hearbeat class has been added allowing users to enable and disable processes using a hearbeat method. In order to use it, within your node create a RosHeartbeat object:

```c++
RosHeartbeat ros_heartbeat("example_heartbeat", 0.1);
```
and use the `enabled` public variable to toggle the node. The enabled variable can be set using a publisher that is running with frequency higher then the one set in the object contructor (`10Hz` above). For example, using bash console:

```sh
rostopic pub /example_heartbeat std_msgs/Bool "data: true" -r 20
```
will set the value to `true`. The value is `true` or `false` depending on the value within the publisher's message. If the heartbeat object doesn't hear anything within it's set timeframe (`0.1c` above), user will be warned and `enabled` will be set to `false`.

## AWS manager

### Getting AWS key

`aws_manager.py` is a script allowing files to be uploaded to and download from AWS. 

In order for the script to work, you first need to get the AWS Access Key. To do that, you need to install your container by running the following command: 
**N.B. This command will overwrite your current container if you have one, if you don't want that to happen, add the name tag to this command. E.g. name=my_new_container**:

```sh
bash <(curl -Ls bit.ly/run-aurora) docker_deploy --read-secure customer_key use_aws=true product=hand_e ethercat_interface=<ethercat_port> config_branch=<demohand_serial> nvidia_docker=true reinstall=true tag=melodic-release image=shadowrobot/dexterous-hand
```

Where:
- <ethercat_port> is the port to which the hand is connected, e.g. enp5s0.
- <demohand_serial> is the configuration branch of the hand, e.g. demohand_D

During installation you will be prompted for a AWS customer key. This key can be retrieved from [here](http://10.5.1.13/mediawiki/index.php/Customer_Keys_for_uploading_ROS_Logs), copy and paste one from the Table of Customer Keys.

If you can't open the link contact the software team at software@shadowrobot.com.

**If you already have a container installed which does not contain an AWS key**, retrieve one of the keys from the link above and within your container run the following command:

```sh
echo "<your_aws_customer_key>" | sudo tee /usr/local/bin/customer.key
```

If you have doubts about this process, contact the software team at software@shadowrobot.com.

### Uploading and downloading files

An example launch file using this script could look like the one below:

```xml
<launch>
  <arg name="download" default="true"/>
  <arg name="upload" default="false"/>
  <arg name="bucket_name" default="shadowrobot.example-bucket"/>

  <arg name="files_base_path" default="$(find example_package)"/>
  <arg name="files_folder_path" default="/example_folder_within_package"/>
  <arg name="file_names" default="[example_file_0, example_file_1]"/>

  <node name="aws_manager_node" pkg="sr_utilities_common" type="aws_manager.py" output="screen">
    <rosparam param="download" subst_value="True">$(arg download)</rosparam>
    <rosparam param="upload" subst_value="True">$(arg upload)</rosparam>
    <rosparam param="files_base_path" subst_value="True">$(arg files_base_path)</rosparam>
    <rosparam param="files_folder_path" subst_value="True">$(arg files_folder_path)</rosparam>
    <rosparam param="bucket_name" subst_value="True">$(arg bucket_name)</rosparam>
    <rosparam param="file_names" subst_value="True">$(arg file_names)</rosparam>
  </node>
```

Launching the above file would upload files `example_file_0` and `example_file_1` located in package `example_package` within folder `example_folder_within_package` to the `shadowrobot.example-bucket` bucket on AWS.

## Conditional Delayed Ros Tool

This script allows to delay the launch of ros launch files and nodes. It checks on the roscore whether the required conditions have been correctly initialized until the timout exceeds.

### Fields

The node **requires** the following parameters:
- package_name: the package in which the executable is stored
- executable_name: could be the name of the roslaunch (without the .launch) or the name of the node
- executable_type: this field has to be set to either **launch** or **node** and defines the type of executable that has to be run
- timeout: after how long the script should give up waiting for the required conditions
- launch_args_list: the list of arguments that has to be passed to the launch file or node

The following are not mandatory and allows to define the conditions necessary to launch the wanted executable:
- topics_list: the list of topics that should be available on roscore before launching
- params_list: the list of parameters that should be available on roscore before launching
- services_list: the list of services that should be advertised before launching

You only need to specify the list you want. For instance, if there are no topics to wait for, you don't need to pass an empty list.

### Real code example

```xml
<node name="conditional_delayed_rostool" pkg="sr_utilities_common" type="conditional_delayed_rostool.py" output="screen">
  <param name="package_name" value="sr_teleop_launch" />
  <param name="executable_name" value="teleop_core_nodes.launch" />
  <param name="executable_type" value="launch" />
  <rosparam param="topics_list">[/rh_trajectory_controller/command, /ros_heartbeat]</rosparam>
  <rosparam param="params_list">[/robot_description]</rosparam>
  <rosparam param="services_list">[/get_planning_scene]</rosparam>
  <param name="launch_args_list" value="sim:=$(arg sim) hand:=$(arg hand) vive:=$(arg vive) jog_arm:=$(arg jog_arm) moveit_arm:=$(arg moveit_arm) moley_arm:=$(arg moley_arm) tracker:=$(arg tracker) tracker_id:=$(arg tracker_id) wrist_wand_id:=$(arg wrist_wand_id) control_wand_id:=$(arg control_wand_id) soft_start_time:=$(arg soft_start_time) local_vive_prefix:=$(arg local_vive_prefix) user_root_tf_name:=$(arg user_root_tf_name) user_forearm_tf_name:=$(arg user_forearm_tf_name) user_wrist_tf_name:=$(arg user_wrist_tf_name) dataflow_handler_config_file_path:=$(arg dataflow_handler_config_file_path) log_topics:='$(arg log_topics)' log_bag_prefix:=$(arg log_bag_prefix) require_trigger:=$(arg require_trigger) require_pedal:=$(arg require_pedal) pedal:=$(arg pedal) side:=$(arg side) wrist_zero:=$(arg wrist_zero)" />
  <param name="timeout" value="60.0" />
</node>
```