# Prophesee ROS Wrapper

![Event-based vision by Prophesee](event-based_vision_PROPHESEE.png)

The aim of this metapackage is wrapping event-based data from [Prophesee sensors](https://www.prophesee.ai/event-based-sensor-packaged) using [OpenEB](https://github.com/prophesee-ai/openeb) software and publishing the event-based data to ROS.

This metapackage contains ROS wrapper (prophesee_ros_driver) and messages (prophesee_event_msgs) for Prophesee sensors.

prophesee_ros_driver is a ROS wrapper including the following nodes:
  * prophesee_ros_publisher - publishing data from Prophesee sensor to ROS topics
  * prophesee_ros_viewer - listening data from ROS topics and visualizing them on a screen

prophesee_event_msgs package contains ROS message types for Prophesee event-based data, including:
  * Event - an event from a Prophesee camera (uint16 x, uint16 y, ros::Time ts, bool polarity)
  * EventArray - a buffer of events (Event[] events)

Supported [Prophesee Evaluation Kit Cameras](https://docs.prophesee.ai/stable/hw/evk/index.html) :
  * EVKV2 - HD
  * EVKV3 - VGA/320/HD
  * EVKV4 - HD

## Requirements

  * Ubuntu 20.04, 22.04 or 24.04 64-bit
  * ROS Noetic
  * [OpenEB](https://github.com/prophesee-ai/openeb)

## Installation

First, retrieve and compile [OpenEB](https://github.com/prophesee-ai/openeb).

Then, compile the packages:

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
  
### Publishing data from a camera and listening to them 

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

  * Update the prophesee_publisher.launch file to set the path to your RAW file (in `raw_file_to_read` parameter)

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

  * Update the prophesee_publisher.launch file to set the path to your RAW file (in `raw_file_to_read` parameter)

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

## Going Further
The ROS wrapper we propose in this repo is a minimal example to get you started.
There are multiple ways to enhance it in terms of features and performance.
The code is open to contributions, thus do not hesitate to [propose pull requests](https://github.com/prophesee-ai/prophesee_ros_wrapper/pulls).

We also recommend you to check the [metavision_ros_driver repo from berndpfrommer](https://github.com/berndpfrommer/metavision_ros_driver)
that offers a ROS driver for event-based cameras with some speed and features enhancements. 

## Contact
In case of any issue, please [raise an issue here on GitHub](https://github.com/prophesee-ai/prophesee_ros_wrapper/issues). 
For any other information [contact us](https://www.prophesee.ai/contact-us/).
