^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prophesee_ros_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2019-10-09)
------------------

0.0.1 (2019-10-09)
------------------
* Merge pull request #5 from a-tonda/hotfix/ubuntu_versions_compatibility
  [FIX] test ubuntu version to set OpenCV path accordingly
* Merge pull request #4 from uzh-rpg/master
  Event messages harmonization with existing event-based sensors within the ROS ecosystem
* Merge branch 'master' of github.com:uzh-rpg/prophesee_ros_wrapper
* [COMPILE] adapt for prophesee release 1.3.0
* prophesee_event_msgs: event buffer with header. events with ros::Time as timestamp
  prophesee_ros_driver: adapt driver and visualizer to the new event messages.
* Merge pull request #3 from uzh-rpg/master
  Include IMU readings
* prophesee_ros_driver: publish IMU events measurements in IMU frame.
  prophesee_ros_driver: publish IMU data by default in launch file
* Initial version
* Contributors: Arnaud Tonda, Javier Hidalgo-Carrió, Natalia Lyubova, meliortony
