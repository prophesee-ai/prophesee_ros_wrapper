# Prophesee ROS Wrapper

![Event-based vision by Prophesee](event-based_vision_PROPHESEE.png)

This metapackage contains ROS driver and messages for Prophesee event-based sensors.
The following packages and nodes are provided:
  * prophesee_ros_driver - ROS driver, including
    * prophesee_ros_publisher - publishing data from Prophesee sensor to ROS topics
    * prophesee_ros_viewer - listening data from ROS topics and visualizing them on a screen
  * prophesee_event_msgs - Prophesee messages:
    * Event - contains an event from a Prophesee camera (uint16 x, uint16 y, bool polarity, ros::Time ts)
    * EventArray - contains a buffer of events (Event[] events)

Supported Prophesee EVK:
  * VGA-CD: PSEE300EVK, PEK3SVCD
  

## Installation

First of all, you would need to install dependencies, such as Metavision SDK:

  * Request an access to Knowledge Center, if not done yet. To get an access, fill the [webform](https://www.prophesee.ai/contact-us/) and provide us a short description of your research project.

  * Sign up for a trial version of [Metavision SDK](https://support.prophesee.ai/portal/en/kb/articles/sdk-trial-request-form), if not done yet.

  * Install Metavision SDK following [the instructions on Knowledge Center](https://support.prophesee.ai/portal/en/kb/articles/linux-software).


Then, compile GitHub code:

  * Clone the source to your catkin workspace ( [create a workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace), if needed)

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
  
The package contains the following ROS nodes:
  * prophesee_ros_publisher
  * prophesee_ros_viewer

### Publishing data from live camera

To publish data from Prophesee camera to ROS topics:

  ```
        roslaunch prophesee_ros_driver prophesee_publisher.launch
  ```

The following topics will be published:
  * /prophesee/camera/camera_info - info about the camera
  * /prophesee/camera/cd_events_buffer - buffer of CD (Change Detection) events
  * /prophesee/camera/imu - IMU data

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

### Publishing data from raw file

To publish data from raw file to ROS topics and view the data:
  * Set the path to your raw file in prophesee_publisher.launch file (raw_file_to_read parameter)
  * Start the viewer, at first:

  ```
        roslaunch prophesee_ros_driver prophesee_viewer.launch
  ```

  * Start the publisher:

  ```
        roslaunch prophesee_ros_driver prophesee_publisher.launch
  ```

### Recording data from raw file to rosbag

To record data from raw file to rosbag:
  * Set the path to your raw file in prophesee_publisher.launch file (raw_file_to_read parameter)
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
The code is open to contributions, so do not hesitate to ask questions, propose pull requests or create bug reports. In case of any issue, please add it here on github. 
For any other information contact us [here](https://www.prophesee.ai/contact-us/) 

