object_detection
================

2 Alternatives to start object_detection:
    - using gazebo to simulate kinect images, as well as objects to be recognized
    - using rosbag of a gazebo simulation

1.) 1)Gazebo simulation (Gazebo v. 1.9 is required for ROS Hydro)
    start gazebo with pr2, a table and objects on the table with the following command:
    roslaunch object_detection pr2_gazebo_table_objects.launch

    2)start rviz with the plugins already loaded and moving arms out of field of view
    roslaunch object_detection pr2_rviz_move_body.launch

    3)Finally launch object_detection (using couchDB) with the filtered kinect (because there
    are some bad images on the simulated kinect topic)
    roslaunch object_detection pr2_filtered_kinect_object_detection.launch 

2.) Record a bag file of the gazebo simulation
    (execute 1.1) and 1.2) )
    Only needed topics are recorded, to keep bag file as small as possible
    rosbag record -o gazebo_bag -e "/head_mount_kinect(.*)|/tf"

    execute command 1.3) for object detection

