#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
//#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( "Position (x, y, z): (" << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z << ") and Orientation: (w, x, y, z): ("
      << feedback->pose.orientation.w << ", " << feedback->pose.orientation.x << ", "
      << feedback->pose.orientation.y << ", " << feedback->pose.orientation.z << ")" );
}

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
//interactive_markers::MenuHandler menu_handler;

// Build the grey box
visualization_msgs::Marker makeBox( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = msg.scale * 0.1;
  marker.scale.y = msg.scale * 0.1;
  marker.scale.z = msg.scale * 0.1;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

// Add control to interactive marker
visualization_msgs::InteractiveMarkerControl& makeBoxControl( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void make6DofMarker( const std::string& frame_id, bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof )
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 0.3;

  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  visualization_msgs::InteractiveMarkerControl control;

  // orientation controls stay fixed.
  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  }

  // If interaction_mode == NONE, it is not possible to move the box using the mouse
  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  // add controls to marker
  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
//  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
//      menu_handler.apply( *server, int_marker.name );
}

int main(int argc, char** argv)
{
  if (argc != 1 && argc != 2)
  {
    ROS_ERROR("Usage: rosrun planning get_position <frame_id>");
    exit(EXIT_FAILURE);
  }

  std::string frame_id;
  if (argc == 1)
    frame_id = "odom_combined";

  if (argc == 2)
    frame_id = argv[1];

  // node name = get_position
  ros::init(argc, argv, "get_position");

  std::string topic_name = "get_position";
  ROS_INFO("Reference frame: %s", frame_id.c_str());
  ROS_INFO("Topic name: /%s", topic_name.c_str());
  if (argc )

  // publishing on topic "get_position"
  server.reset( new interactive_markers::InteractiveMarkerServer(topic_name,"",false) );

  tf::Vector3 position;
  position = tf::Vector3(1, 1, 1);

  // Box with circles, can only be moved using the circles and arrows.
  //make6DofMarker(frame_id, false, visualization_msgs::InteractiveMarkerControl::NONE, position, true );

  // Box with circles, can only be moved using the circles and arrows. BUT with the exception that the
  // orientation of the controls will stay fixed, independent of the orientation of the frame being controlled.
  //make6DofMarker(frame_id, true, visualization_msgs::InteractiveMarkerControl::NONE, position, true );

  // Box without circles, box can be moved using the mouse
  //make6DofMarker(frame_id, false, visualization_msgs::InteractiveMarkerControl::MOVE_3D, position, false );

  // Box without circles, box is rotation using the mouse
  //make6DofMarker(frame_id, false, visualization_msgs::InteractiveMarkerControl::ROTATE_3D, position, false );

  // Box with circles, box can be moved with the mouse and rotated using the circles
  make6DofMarker(frame_id, false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true );
  server->applyChanges();


  // start the ROS main loop
  ros::spin();
}
