#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "mobile_manipulation_alg/vision_command.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Point.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf/transform_datatypes.h"


#define MODE_1 1
#define MODE_2 2



typedef struct value_range
{
  float min;
  float max;
}value_range_t;

/*flags*/
bool stop_robot = false;
bool previous_open_mouth_commmand = true;
/* maximum values*/
// value_range_t husky_trans_speed = {0, 0.5}; // 1m/s
// value_range_t husky_rot_speed = {0,2};
// value_range_t unit_range = {0,1};
// value_range_t negative_unit_range = {-1,0};

/*global variable*/
mobile_manipulation_alg::vision_command vision_command_;
/*publisher*/
ros::Publisher *husky_wheel_pub_ptr;
cv::Mat image_depth_raw;
cv::Mat image_raw;
geometry_msgs::TransformStamped transformStamped;

float depth = 0;
int x_cursor_pos = 0;
int y_cursor_pos = 0;

/* depth information*/
typedef union U_FloatParse {
    float float_data;
    unsigned char byte_data[4];
} U_FloatConvert;

/*functions*/
// float scale_value(value_range_t input_range,value_range_t output_range, float value);
void move_base(float rotation, float translation);
void stop_base();
/*callback*/
void visionCommandCallback(const mobile_manipulation_alg::vision_command  msg);
void imageAndDepthCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::ImageConstPtr& depth_msg);
void gotopose(geometry_msgs::Point pose_end_effector);
bool getToolPosition(tf2_ros::Buffer *tfBuffer_ptr);
int sgn(float val);
int ReadDepthData(unsigned int height_pos, unsigned int width_pos, sensor_msgs::ImageConstPtr depth_image);

int main(int argc, char** argv)
{

  ros::init(argc, argv, "wheel_chair_head_control_manager_node");
  ros::NodeHandle nh;
  /* arm planning part*/
  ros::AsyncSpinner spinner(1);
  spinner.start();
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener_ptr(tfBuffer);
  static const std::string PLANNING_GROUP = "ur5_arm";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  // const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("ur5_arm_base_link");
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  visual_tools.deleteAllMarkers();
  move_group_interface.setGoalTolerance(0.01);
  move_group_interface.setPoseReferenceFrame("ur5_arm_base_link");
  /*husky_wheel_publisher*/
  husky_wheel_pub_ptr = new ros::Publisher(nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1));
  image_transport::ImageTransport it_(nh);
  typedef image_transport::SubscriberFilter ImageSubscriber;
  ImageSubscriber image_sub_(it_,"/camera1/image_raw",1);
  ImageSubscriber depth_sub_(it_,"/camera1/depth/image_raw",1);
  ros::Publisher tube_info_pub;
  /* synchronize the depth and image callback to ensure depth data corresponds to the pixel reading*/
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer< MySyncPolicy> sync(MySyncPolicy(10), image_sub_, depth_sub_);
  sync.registerCallback( boost::bind(imageAndDepthCb, _1, _2 ) );
  cv::namedWindow("Image window",cv::WINDOW_NORMAL);
  // cv::namedWindow("depth window",cv::WINDOW_NORMAL);
  bool test_mouth_node = true;
  /* vision command node Subscriber*/
  ros::Subscriber vision_command_sub = nh.subscribe("/vision/command", 1, visionCommandCallback);

  vision_command_.mode = (int)MODE_1;

  ros::Rate loop_rate(50);
  geometry_msgs::Point pose_offset;
  while(ros::ok() && !stop_robot)
  {

    stop_robot = vision_command_.emergency_stop;
    if(vision_command_.mode == MODE_1)
    {
      /*moving base*/
      /* head rotation is switch be carefull*/
      if(vision_command_.open_mouth)
      {
        move_base(vision_command_.yaw, vision_command_.pitch);
      }
    }
    else if(vision_command_.mode == MODE_2)
    {
      stop_base();
      if(vision_command_.open_mouth)
      {
        // pose_offset.x =(float) image_depth_raw.at<float>(vision_command_.y_cursor,vision_command_.x_cursor);
        // std::cout << depth << std::endl;
         bool arm_process_finished = false;

        while(!arm_process_finished && !stop_robot)
        {
          geometry_msgs::Twist cmd_vel;
          float final_depth;
          if(depth == -1)
          {
            //to near move base back
            // move_base(0, 0.8);
            /* scaled speed*/
            cmd_vel.linear.x = -0.1;

            husky_wheel_pub_ptr->publish(cmd_vel);
          }
          else if(depth > 450)
          {
            // to near move base forward
            cmd_vel.linear.x = 0.1;

            husky_wheel_pub_ptr->publish(cmd_vel);
            // move_base(0,-0.5);
          }
          else
          {
            /* convert 2d point to 3d point*/
            float X = (x_cursor_pos-320.5)/524.2422531097977;
            float Y = (y_cursor_pos-240.5)/524.2422531097977;
            std::cout << depth << std::endl;
            pose_offset.x = depth/1000.0;
            pose_offset.y = -X*pose_offset.x;
            pose_offset.z = -Y*pose_offset.x;
            geometry_msgs::Point pose_offset2;
            pose_offset2.x  = (pose_offset.x-50)/1000;
            // pose_offset.x = 0;
            // gotopose(pose_offset);
            // /*go to grasp pose*/
            // gotopose(pose_offset2);
            // pose_offset2.x = pose_offset2.x*-1;
            // // go back to pregrasp
            // gotopose(pose_offset2);
            geometry_msgs::Pose target_pose1;
            std::vector<geometry_msgs::Pose> waypoints;
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0;
            const double eef_step = 0.01;
            int time_count = 0;
            geometry_msgs::Pose target_pose3;
            double fraction;
            std::cout << pose_offset.x << " " << pose_offset.y << pose_offset.z << std::endl;

            ROS_INFO("Going to first point");

            if(pose_offset.y > 0.03 || pose_offset.z > 0.03 ||pose_offset.x > 0.03  )
            {
              while(!getToolPosition(&tfBuffer) && time_count!=100 )
              {
                time_count+=1;
                // std::cout << time_count << std::endl;
              }
              // target_pose1.orientation.w = transformStamped.transform.rotation.w;
              // target_pose1.orientation.x = transformStamped.transform.rotation.x;
              // target_pose1.orientation.y = transformStamped.transform.rotation.y;
              // target_pose1.orientation.z = transformStamped.transform.rotation.z;
              target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

              target_pose1.position.x = transformStamped.transform.translation.x;
              target_pose1.position.y = transformStamped.transform.translation.y;
              target_pose1.position.z = transformStamped.transform.translation.z;
              // waypoints.push_back(target_pose1);
              target_pose3 = target_pose1;
              target_pose3.position.y += pose_offset.y +0.05;
              target_pose3.position.z += pose_offset.z +0.05;
              waypoints.push_back(target_pose3);  // down
              target_pose3.position.x += (pose_offset.x -0.1);
              waypoints.push_back(target_pose3);  // down
              target_pose3.position.x -= (pose_offset.x -0.1);
              waypoints.push_back(target_pose3);  // down
              target_pose3.position.y -= pose_offset.y +0.05;
              target_pose3.position.z -= pose_offset.z +0.05;
              waypoints.push_back(target_pose3);  // down

              std::cout << move_group_interface.getPlanningFrame();
              fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
              // Visualize the plan in RViz
               visual_tools.deleteAllMarkers();
               visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
               visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
               for (std::size_t i = 0; i < waypoints.size(); ++i)
               {
                 visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
                 std::cout << i << std::endl;
               }
               visual_tools.trigger();
               move_group_interface.execute(trajectory);
               ROS_INFO("Picked finished!!!!!!, stoping demo");
               break;
               arm_process_finished = false;
            }
            else
            {
              ROS_INFO("You are already close to your goal");
            }
              test_mouth_node = false;
          }
        }


      }
      else
      {
        // ROS_INFO("object out of robot reach");
      }
      }

    ros::spinOnce();
    loop_rate.sleep();

    cv::waitKey(1);

  }

  cv::destroyWindow("Image window");
 // cv::destroyWindow("depth window");
  return 0;
}


// float scale_value(value_range_t input_range,value_range_t output_range, float value)
// {
//   // assumption that value is between 0 and 1
//   float inter_input = (input_range.max-input_range.min);
//   float inter_output = (output_range.max-output_range.min);
//   float offset_input = value-input_range.min;
//
//   return (output_range.min + ((offset_input)/(inter_input))*inter_output);
// }

int sgn(float val) {
    return (float(0) < val) - (val < float(0));
}

/*callback*/
void visionCommandCallback(const mobile_manipulation_alg::vision_command  msg)
{
  // vision_command =msg;
  if(vision_command_.mode == MODE_2)
  {
    if(!vision_command_.open_mouth)
    {
      x_cursor_pos = vision_command_.x_cursor;
      y_cursor_pos = vision_command_.y_cursor;
    }


  }
  vision_command_ = msg;

}
void move_base(float rotation, float translation)
{
  float rot = 0;
  float trans = 0;
  int sign_rot = sgn(-rotation);
  int sign_tra = sgn(translation);

  if(std::abs(rotation)> 13)
  {
    rot = 1.1;
  }
  if(std::abs(translation)> 4)
  {
    trans = 1;
  }
  geometry_msgs::Twist cmd_vel;
  /* scaled speed*/
  cmd_vel.linear.x = (sign_tra)*trans;
  cmd_vel.angular.z = (sign_rot)*rot;
  husky_wheel_pub_ptr->publish(cmd_vel);

}
void stop_base()
{
 geometry_msgs::Twist stop_vel;
 husky_wheel_pub_ptr->publish(stop_vel);

}
void imageAndDepthCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::ImageConstPtr& depth_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  /*for the depth information*/
  cv_bridge::CvImagePtr cv_ptr_depth;
  try
  {
    cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  image_depth_raw = cv_ptr_depth->image;
  image_raw = cv_ptr->image;
  depth = ReadDepthData(vision_command_.y_cursor,vision_command_.x_cursor, depth_msg);
  if(vision_command_.mode == MODE_2)
  {
    // if(!vision_command_.open_mouth)
    // {
      cv::circle(cv_ptr->image, cv::Point(x_cursor_pos, y_cursor_pos), 10,cv::Scalar(255,0,0),cv::FILLED);
    // }
    // previous_open_mouth_commmand = vision_command_.open_mouth;

  }
  // std::cout << vision_command_.x_cursor << vision_command_.x_cursor <<std::endl;

  cv::imshow("Image window",image_raw);
  // cv::imshow("depth window", image_depth_raw);

}
void gotopose(geometry_msgs::Point pose_end_effector)
{
  std::cout<<pose_end_effector.x << " " << pose_end_effector.y << " " << pose_end_effector.z << std::endl;

}
int ReadDepthData(unsigned int height_pos, unsigned int width_pos, sensor_msgs::ImageConstPtr depth_image)
{
    // If position is invalid
    if ((height_pos >= depth_image->height) || (width_pos >= depth_image->width))
        return -1;
    int index = (height_pos*depth_image->step) + (width_pos*(depth_image->step/depth_image->width));
    // If data is 4 byte floats (rectified depth image)
    if ((depth_image->step/depth_image->width) == 4) {
        U_FloatConvert depth_data;
        int i, endian_check = 1;
        // If big endian
        if ((depth_image->is_bigendian && (*(char*)&endian_check != 1)) ||  // Both big endian
           ((!depth_image->is_bigendian) && (*(char*)&endian_check == 1))) { // Both lil endian
            for (i = 0; i < 4; i++)
                depth_data.byte_data[i] = depth_image->data[index + i];
            // Make sure data is valid (check if NaN)
            if (depth_data.float_data == depth_data.float_data)
                return int(depth_data.float_data*1000);
            return -1;  // If depth data invalid
        }
        // else, one little endian, one big endian
        for (i = 0; i < 4; i++)
            depth_data.byte_data[i] = depth_image->data[3 + index - i];
        // Make sure data is valid (check if NaN)
        if (depth_data.float_data == depth_data.float_data)
            return int(depth_data.float_data*1000);
        return -1;  // If depth data invalid
    }
    // Otherwise, data is 2 byte integers (raw depth image)
   int temp_val;
   // If big endian
   if (depth_image->is_bigendian)
       temp_val = (depth_image->data[index] << 8) + depth_image->data[index + 1];
   // If little endian
   else
       temp_val = depth_image->data[index] + (depth_image->data[index + 1] << 8);
   // Make sure data is valid (check if NaN)
   if (temp_val == temp_val)
       return temp_val;
   return -1;  // If depth data invalid
}
bool getToolPosition(tf2_ros::Buffer *tfBuffer_ptr)
{
  try
  {
      transformStamped = tfBuffer_ptr->lookupTransform("ur5_arm_base_link", "ur5_arm_ee_link",ros::Time(0));
          return true;
  }

  catch (tf2::TransformException &ex)
  {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
          return false;
  }

}
