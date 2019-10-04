# Prophesee ROS Wrapper

![Event-based vision by Prophesee](event-based_vision_PROPHESEE.png)

This metapackage contains ROS driver and messages for Prophesee event-based sensors.
The following packages and nodes are provided:
  * prophesee_ros_driver - ROS driver (a wrapper around Prophesee Driver), including
    * prophesee_ros_publisher - publishing data from Prophesee sensor to ROS topics
    * prophesee_ros_viewer - listening data from ROS topics and visualizing them on a screen
  * prophesee_event_msgs - Prophesee messages:
    * Event - contains an event from a Prophesee camera (uint16 x, uint16 y, bool polarity, ros::Time ts)
    * EventArray - contains a buffer of events (Event[] events)

Supported Prophesee EVK:
  * VGA-CD: PSEE300EVK, PEK3SVCD
  * HVGA-EM: PSEE350EVK, PEK3SHEM
  

## Installation

  * First of all, you would need to install dependencies, such as Prophesee Driver SDK. Prophesee Driver SDK can be downloaded from our Knowledge Center. In case if you do not have an access to Knowledge Center yet, then please provide us a short decription of your research project and request an access via this webform https://www.prophesee.ai/contact-us/

    ```
        sudo apt install prophesee-*
    ```

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
  
prophesee_ros_driver package contains the following ROS nodes:
  * prophesee_ros_publisher
  * prophesee_ros_viewer

### Data publisher

To publish data from a Prophesee camera to ROS topics:

  ```
        roslaunch prophesee_ros_driver prophesee_publisher.launch
  ```

The following topics will be published:
  * camera_info - info about the camera
  * cd_events_buffer - buffer of CD (Change Detection) events
  * graylevel_image - Gray-level frame reconstructed from EM and CD events
 
 

### Data viewer

To visualize data from ROS topics:

  ```
        roslaunch prophesee_ros_driver prophesee_viewer.launch
  ```

## Contact
The code is open to contributions, so do not hesitate to ask questions, propose pull requests or create bug reports. In case of any issue, please add it here on github. 
For any other information contact us [here](https://www.prophesee.ai/contact-us/) 

