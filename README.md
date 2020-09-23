# Prophesee ROS Wrapper

![Event-based vision by Prophesee](event-based_vision_PROPHESEE.png)

This metapackage contains ROS driver and messages for [Prophesee](https://www.prophesee.ai) event-based sensors.

The aim of this metapackage is wrapping data access from event-based sensors from [Metavision SDK](https://docs.prophesee.ai/) software and publishing the data to ROS.

The following packages and nodes are provided:
  * prophesee_ros_driver - ROS driver, including
    * prophesee_ros_publisher - publishing data from Prophesee sensor to ROS topics
    * prophesee_ros_viewer - listening data from ROS topics and visualizing them on a screen
  * prophesee_event_msgs - Prophesee messages:
    * Event - contains an event from a Prophesee camera (uint16 x, uint16 y, ros::Time ts, bool polarity)
    * EventArray - contains a buffer of events (Event[] events)

Supported Prophesee EVK:
  * EVK Gen3
  * other cameras supported by Metavision SDK software
  

## Installation

First of all, install Metavision SDK following [these instructions](https://docs.prophesee.ai/getting_started/installation/linux.html).

Then, compile the wrapper code:

  * Clone the source code to your catkin workspace ( [create a workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace), if needed)

    ```
        cd catkin_ws/src
        git clone https://github.com/prophesee-ai/prophesee_ros_wrapper.git
        cd ..
    ```

  * Compile

    ```
        catkin_make
    ```

  * Source the workspace

    ```
        source ~/catkin_ws/devel/setup.bash
    ```
  
  

## Getting Started
  
### Publishing data from live camera

To publish data from Prophesee camera to ROS topics:

  ```
        roslaunch prophesee_ros_driver prophesee_publisher.launch
  ```

The following topics will be published:
  * /prophesee/camera/camera_info - info about the camera
  * /prophesee/camera/cd_events_buffer - buffer of CD (Change Detection) events

### Viewing data from ROS topics

To visualize data from ROS topics:

  ```
        roslaunch prophesee_ros_driver prophesee_viewer.launch
  ```

### Recording data from live camera to rosbag

To record data from live camera to rosbag:
  * Start the publisher:

  ```
        roslaunch prophesee_ros_driver prophesee_publisher.launch
  ```

  * Start rosbag recording (choose the topics to record or record all available topics):

  ```
        rosbag record -a
  ```

### Publishing data from RAW file

To publish data from RAW file to ROS topics and view the data:
  * Open the prophesee_publisher.launch file and set the path to your RAW file (in raw_file_to_read parameter)

  ```
        rosed prophesee_ros_driver prophesee_publisher.launch
  ```

  * Start the viewer, at first:

  ```
        roslaunch prophesee_ros_driver prophesee_viewer.launch
  ```

  * Start the publisher:

  ```
        roslaunch prophesee_ros_driver prophesee_publisher.launch
  ```

Note that before starting the publisher, you need to start the viewer node (like here) or ROS core.

At the end of the RAW file, the publisher will stop on its own, but the viewer won't stop, so it's up to you to quite the viewer.

### Recording data from RAW file to rosbag

To record data from RAW file to rosbag:
  * Open the prophesee_publisher.launch file and set the path to your RAW file (in raw_file_to_read parameter)

  ```
        rosed prophesee_ros_driver prophesee_publisher.launch
  ```

  * Start the ROS core

  ```
        roscore
  ```

  * Start rosbag recording (choose the topics to record or record all available topics):

  ```
        rosbag record -a
  ```

  * Start the publisher:

  ```
        roslaunch prophesee_ros_driver prophesee_publisher.launch
  ```

## Contact
The code is open to contributions, thus do not hesitate to propose pull requests or create/fix bug reports.
In case of any issue, please add it here on GitHub. 
For any other information contact us [here](https://www.prophesee.ai/contact-us/) 

