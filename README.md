# Prophesee ROS Wrapper

![Event-based vision by Prophesee](event-based_vision_PROPHESEE.png)

This is a metapackage containing ROS wrapper and messages for [Prophesee](https://www.prophesee.ai) event-based sensors.

The aim of this metapackage is wrapping data access from event-based sensors using [OpenEB](https://github.com/prophesee-ai/openeb) and publishing the data to ROS.

The following packages and nodes are included:
  * prophesee_ros_driver - main ROS wrapper, including
    * prophesee_ros_publisher - publishing data from Prophesee sensor to ROS topics
    * prophesee_ros_viewer - listening data from ROS topics and visualizing them on a screen
  * prophesee_event_msgs - Prophesee messages, including
    * Event - containing an event from a Prophesee camera (uint16 x, uint16 y, ros::Time ts, bool polarity)
    * EventArray - containing a buffer of events (Event[] events)

Supported [Prophesee sensors](https://docs.prophesee.ai/2.2.1/hw/evk/index.html) :
  * EVKV1 Gen3.0, Gen3.1, Gen4.0, Gen4.1
  * EVKV2 Gen4
  * EVKV3 Gen31, Gen41

## Requirements

  * Ubuntu 20.04 or 18.04
  * ROS Noetic or ROS Melodic
  * [OpenEB](https://github.com/prophesee-ai/openeb) - starting from v2.2.0

## Installation

First of all, retrieve and compile [OpenEB](https://github.com/prophesee-ai/openeb).

If you want to use the ROS wraper with live Prophesee cameras, then install Prophesee plugins as described in [Prophesee documentation](https://docs.prophesee.ai/2.2.1/installation/linux_open_from_source.html#chapter-installation-linux-open-from-source).

Then, compile the wrapper code:

  * Clone the source code to your catkin workspace ([create a workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace), if needed)

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
  
### Publishing data from a live camera and listening them 

To publish data from Prophesee camera to ROS topics, run:

  ```
        roslaunch prophesee_ros_driver prophesee_publisher.launch
  ```

The following topics will be published:

  * /prophesee/camera/cd_events_buffer - buffer of CD (Change Detection) events
  * /prophesee/camera/camera_info - information about the camera

To listen data from ROS topics and visualize them:

  ```
        roslaunch prophesee_ros_driver prophesee_viewer.launch
  ```

### Recording data from a live camera to rosbag

To record data from live camera to rosbag:
  * Start the publisher:

  ```
        roslaunch prophesee_ros_driver prophesee_publisher.launch
  ```

  * Start rosbag recording (choose the topics to record or record all available topics):

  ```
        rosbag record -a
  ```

### Publishing data from a RAW file

To publish data from RAW file to ROS topics and view the data:

  * Update the prophesee_publisher.launch file to set the path to your RAW file (i.e raw_file_to_read parameter)

  ```
        rosed prophesee_ros_driver prophesee_publisher.launch
  ```

  * Start the ROS core

  ```
        roscore
  ```
  
  * Start the viewer, at first to be sure to not miss any data:

  ```
        roslaunch prophesee_ros_driver prophesee_viewer.launch
  ```

  * Start the publisher:

  ```
        roslaunch prophesee_ros_driver prophesee_publisher.launch
  ```

At the end of the RAW file, the publisher will stop on its own, but the viewer won't stop, so it's up to you to quit the viewer.

### Recording data from a RAW file to rosbag

To record data from RAW file to rosbag:

  * Update the prophesee_publisher.launch file to set the path to your RAW file (in raw_file_to_read parameter)

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
For any other information [contact us](https://www.prophesee.ai/contact-us/).

