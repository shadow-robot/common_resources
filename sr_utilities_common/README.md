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
