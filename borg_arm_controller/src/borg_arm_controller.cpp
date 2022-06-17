// COPYRIGHT Pedro Tecchio and Anderson Queiroz 2020

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <geometric_shapes/shape_operations.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <std_srvs/Trigger.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <tf/tf.h>

#include <math.h>

#define SLEEP_AT_STATE 3
#define MAX_ALLOWED_JOINT_ERROR 0.009
#define MAX_ALLOWED_EXECUTION_TIME 50.0
#define MAX_ALLOWED_PLANNING_TIME 20
// #define GOAL_TOLERANCE 0.008
#define GOAL_POSITION_TOLERANCE 0.001
#define GOAL_ORIENTATION_TOLERANCE 0.1

#define AT_HOME            0
#define WAIT_START         1
#define AT_NORTH           2
#define AT_NORTHWEST       3
#define AT_SOUTHWEST       4
#define AT_NORTHWEST2      5
#define AT_NORTH2          6
#define AT_NORTHEAST       7
#define AT_SOUTHEAST       8

#define TAG_NOT_FOUND     10
#define AT_NEAR_BUTTON    12
#define AT_BUTTON         13
#define AT_LEAVING_BUTTON 14
#define AT_HOME_AND_END   15


typedef const std::map<std::string, std::vector<double>> positions_map;

class BorgArmController {
 private:
  bool start_scan_;
  int state_;
  double elapsed_time_sec_;
  int box_orientation_;
  std::stringstream ss_;
  std::ofstream out_file_;
  double offset_x_, offset_y_, offset_z_;
  double offset_orientation_x_, offset_orientation_y_, offset_orientation_z_;

  // ROS
  ros::Time time_begin_, time_end_, time_scan_, time_button_;
  ros::NodeHandle nh_;
  ros::ServiceServer start_scan_service_;

  // TFs
  tf::TransformListener listener_;
  std::vector<tf::StampedTransform> tf_buttons_;
  std::vector<tf::StampedTransform> tf_boxes_;
  std::vector<geometry_msgs::Pose> pose_buttons_;
  std::vector<geometry_msgs::Pose> pose_boxes_;
  geometry_msgs::Pose pose_target_, pose_achieved_;

  // MoveIt
  const std::string PLANNING_GROUP_ = "borg_arm";
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  const robot_state::JointModelGroup* joint_model_group_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;
  moveit::planning_interface::MoveGroupInterface::Plan plan_;

  Eigen::Isometry3d text_pose_;

  moveit::core::RobotStatePtr current_state_ptr_;

  // Collisions
  shapes::Mesh* mesh_path_;
  shape_msgs::Mesh box_;
  std::vector<moveit_msgs::CollisionObject> collision_objects_workbench_;
  std::vector<moveit_msgs::CollisionObject> collision_objects_boxes_;


  // constants
  const std::vector<std::string> tf_id_buttons_ = {
    "near_button", "pressed_button"};
  const std::vector<std::string> tf_id_boxes_ = {"box"};
  const std::string object_id_workbench_ = "workbench";
  const std::vector<double> dimensions_workbench_ = {0.803, 1.603, 0.0};
  const std::vector<std::string> object_id_boxes_ = {"box"};
  // const std::vector<std::vector<double>> dimensions_boxes_ = {
  //   {0.2, 0.2, 0.15}, {0.2, 0.15, 0.13}, {0.2, 0.2, 0.02}};


  const double d2r_ = {M_PI/180};
  positions_map joint_positions_ = {
    {"home",       {0,  0,   0, 0, 0}},
    {"rest",       {0, toRad(90), toRad(-90), 0, 0}},
    {"north",      {0, toRad(50), toRad(-85), 0, 0}},
    {"north2",     {toRad(-20), toRad(40), toRad(-85), 0, 0}},
    {"northwest",  {toRad(-60), toRad(45), toRad(-45), 0, 0}},
    {"northeast",  {toRad(60), toRad(45), toRad(-45), 0, 0}},
    {"southwest",  {toRad(-130), toRad(45), toRad(-45), 0, 0}},
    {"southeast",  {toRad(130), toRad(45), toRad(-45), 0, 0}},
    {"northwest2", {toRad(-60), toRad(35), toRad(-45), 0, 0}},
    {"northeast2", {toRad(60), toRad(35), toRad(-45), 0, 0}}};


  // methods
  void init_rviz();
  void init_collision();
  void setCollision(
    std::vector<moveit_msgs::CollisionObject> *collision_objects_msg,
    std::string _id,
    std::vector<double> _dimensions,
    geometry_msgs::Pose _pose);
  void setCollisionMesh(
    std::vector<moveit_msgs::CollisionObject> *collision_objects_msg,
    std::string _id,
    geometry_msgs::Pose _pose);
  void putCollisionBoxes();
  bool moveByJointAngle(std::string _position);
  bool moveByJointAngle(std::vector<double> _joint_group_positions);
  bool moveByJointAngleNoPlan(std::string _position);
  bool moveByJointAngleNoPlan(std::vector<double> _joint_group_positions);
  bool moveByPose(geometry_msgs::Pose _pose);
  bool moveByPoseNoTrajectory(geometry_msgs::Pose _pose);
  bool checkJointPositions(std::vector<double> _joint_values);
  void copyTf2PoseMsg(tf::StampedTransform* _tf, geometry_msgs::Pose* _pose);
  bool findTag();
  bool startServiceCallback(
    std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);


  inline double toDeg(double radians) {
    return radians * (180.0 / M_PI);}
  inline double toRad(double degrees) {
    return degrees * (M_PI / 180.0);}


 public:
  BorgArmController(ros::NodeHandle* _nh);
  ~BorgArmController();
  bool stateMachine();
};

BorgArmController::BorgArmController(ros::NodeHandle* _nh):
  nh_(*_nh), move_group_(PLANNING_GROUP_), visual_tools_("world") {
  joint_model_group_ =
    move_group_.getCurrentState()->getJointModelGroup(PLANNING_GROUP_);

  //Path where the .dae or .stl object is located
  mesh_path_ = shapes::createMeshFromResource("package://borg_arm_controller/stl/Collision_box.stl");
  ROS_INFO("Your mesh was loaded");

  tf_boxes_.resize(tf_id_boxes_.size());
  tf_buttons_.resize(tf_id_buttons_.size());
  pose_boxes_.resize(tf_id_boxes_.size());
  pose_buttons_.resize(tf_id_buttons_.size());

  offset_x_ = 0.0;
  offset_y_ = 0.0;
  offset_z_ = 0.0;

  offset_orientation_x_ = 0.0;
  offset_orientation_y_ = 0.0;
  offset_orientation_z_ = 0.0;

  init_rviz();
  init_collision();

  start_scan_ = false;
  start_scan_service_ = nh_.advertiseService("StartScanService",
                                             &BorgArmController::startServiceCallback,
                                             this);
  state_ = AT_HOME;
  out_file_.open("~/Projects/borg_arm/test_time.txt", std::ofstream::out | std::ofstream::app);
}

BorgArmController::~BorgArmController() {
  out_file_.close();
}


bool BorgArmController::startServiceCallback(
  std_srvs::TriggerRequest& request,
  std_srvs::TriggerResponse& response) {
  // start_scan_ = ~start_scan_;
  response.success = true;
  if (!start_scan_) {
    start_scan_ = true;
    ROS_WARN_STREAM("Service is " << start_scan_);
    response.message = "Scan started!";
  } else {
    response.message = "Scan has already started!";
  }
  return true;
}

void BorgArmController::init_rviz() {
  // Visual tools
  visual_tools_.deleteAllMarkers();
  visual_tools_.loadRemoteControl();

  text_pose_ = Eigen::Isometry3d::Identity();
  text_pose_.translation().z() = 1.75;

  visual_tools_.publishText(
    text_pose_,
    "Borg_arm BorgArmController",
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);

  visual_tools_.trigger();
  ROS_INFO_NAMED("Reference frame: %s",
    move_group_.getPlanningFrame().c_str());
  ROS_INFO_NAMED("End effector link: %s",
    move_group_.getEndEffectorLink().c_str());
}

void BorgArmController::init_collision() {
  // Workbench object
  geometry_msgs::Pose pose_workbench;
  pose_workbench.orientation.w = 0.0;
  pose_workbench.position.x = 0.0;
  pose_workbench.position.y = 0.0;
  pose_workbench.position.z = 0.85;
  setCollision(&collision_objects_workbench_,
    object_id_workbench_, dimensions_workbench_, pose_workbench);

  // Init box mesh
  shapes::ShapeMsg box_msg;
  shapes::constructMsgFromShape(mesh_path_, box_msg);
  box_ = boost::get<shape_msgs::Mesh>(box_msg);
}

void BorgArmController::setCollision(
  std::vector<moveit_msgs::CollisionObject> *collision_objects_msg,
  std::string _id,
  std::vector<double> _dimensions,
  geometry_msgs::Pose _pose) {
  // creates basic msg types
  moveit_msgs::CollisionObject collision_object_msg;
  shape_msgs::SolidPrimitive solid_primitive;
  // fill collision object msg
  collision_object_msg.header.frame_id = move_group_.getPlanningFrame();
  collision_object_msg.id = _id;
  // fill primitive type
  solid_primitive.type = solid_primitive.BOX;
  solid_primitive.dimensions.resize(_dimensions.size());
  for (size_t i = 0; i < _dimensions.size(); i++) {
    solid_primitive.dimensions[i] = _dimensions[i];
  }
  // push data to objects
  collision_object_msg.primitives.push_back(solid_primitive);
  collision_object_msg.primitive_poses.push_back(_pose);
  collision_object_msg.operation = collision_object_msg.ADD;

  collision_objects_msg->push_back(collision_object_msg);
}

void BorgArmController::setCollisionMesh(
  std::vector<moveit_msgs::CollisionObject> *collision_objects_msg,
  std::string _id,
  geometry_msgs::Pose _pose) {
  // creates basic msg types
  moveit_msgs::CollisionObject collision_object_msg;
  // fill collision object msg
  collision_object_msg.header.frame_id = move_group_.getPlanningFrame();
  collision_object_msg.id = _id;

  // push data to objects
  collision_object_msg.meshes.resize(1);
  collision_object_msg.mesh_poses.resize(1);
  collision_object_msg.meshes.push_back(box_);
  collision_object_msg.mesh_poses.push_back(_pose);
  collision_object_msg.operation = collision_object_msg.ADD;
  collision_objects_msg->push_back(collision_object_msg);
}


void BorgArmController::putCollisionBoxes() {
  collision_objects_boxes_.clear();
  for (size_t i = 0; i < object_id_boxes_.size(); i++) {
    setCollisionMesh(&collision_objects_boxes_,
      object_id_boxes_[i], pose_boxes_[i]);
  }
  planning_scene_interface_.addCollisionObjects(collision_objects_boxes_);
}

bool BorgArmController::moveByJointAngle(std::string _position) {
  bool success = {false};
  double secs = ros::Time::now().toSec() + MAX_ALLOWED_EXECUTION_TIME;
  std::vector<double> joint_group_positions;

  current_state_ptr_ = move_group_.getCurrentState();
  move_group_.setStartStateToCurrentState();


  joint_group_positions = joint_positions_.at(_position);

  move_group_.setJointValueTarget(joint_group_positions);
  // move_group_.setGoalTolerance(GOAL_TOLERANCE);
  // move_group_.setGoalPositionTolerance(GOAL_POSITION_TOLERANCE);
  // move_group_.setGoalOrientationTolerance(GOAL_ORIENTATION_TOLERANCE);
  move_group_.setPlanningTime(MAX_ALLOWED_PLANNING_TIME);

  success = (move_group_.plan(plan_) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  joint_group_positions = plan_.trajectory_.joint_trajectory.points[
    plan_.trajectory_.joint_trajectory.points.size()-1].positions;

  if (success) {
    // move_group_.execute(plan_);
    while (secs > ros::Time::now().toSec()) {
      if (checkJointPositions(joint_group_positions))
        return true;
    }
  }
  return false;
}

bool BorgArmController::moveByJointAngle(
  std::vector<double> _joint_group_positions) {
  bool success = {false};
  double secs = ros::Time::now().toSec() + MAX_ALLOWED_EXECUTION_TIME;
  std::vector<double> joint_group_positions;

  current_state_ptr_ = move_group_.getCurrentState();
  move_group_.setStartStateToCurrentState();

  joint_group_positions = _joint_group_positions;

  move_group_.setJointValueTarget(joint_group_positions);
  // move_group_.setGoalTolerance(GOAL_TOLERANCE);
  // move_group_.setGoalPositionTolerance(GOAL_POSITION_TOLERANCE);
  // move_group_.setGoalOrientationTolerance(GOAL_ORIENTATION_TOLERANCE);
  // move_group_.setPlanningTime(MAX_ALLOWED_PLANNING_TIME);

  success = (move_group_.plan(plan_) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  joint_group_positions = plan_.trajectory_.joint_trajectory.points[
    plan_.trajectory_.joint_trajectory.points.size()-1].positions;

  if (success) {
    // move_group_.execute(plan_);
    while (secs > ros::Time::now().toSec()) {
      if (checkJointPositions(joint_group_positions))
        return true;
    }
  }
  return false;
}


bool BorgArmController::moveByJointAngleNoPlan(std::string _position) {
  bool success = {false};
  double secs = ros::Time::now().toSec() + MAX_ALLOWED_EXECUTION_TIME;
  std::vector<double> joint_group_positions;

  current_state_ptr_ = move_group_.getCurrentState();
  move_group_.setStartStateToCurrentState();

  // update joint positions
  joint_group_positions = joint_positions_.at(_position);

  move_group_.setJointValueTarget(joint_group_positions);

  move_group_.move();
  while (secs > ros::Time::now().toSec()) {
    if (checkJointPositions(joint_group_positions))
      return true;
  }
  return false;
}

bool BorgArmController::moveByJointAngleNoPlan(
  std::vector<double> _joint_group_positions) {
  bool success = {false};
  double secs = ros::Time::now().toSec() + MAX_ALLOWED_EXECUTION_TIME;

  current_state_ptr_ = move_group_.getCurrentState();
  move_group_.setStartStateToCurrentState();

  move_group_.setJointValueTarget(_joint_group_positions);

  move_group_.move();
  while (secs > ros::Time::now().toSec()) {
    if (checkJointPositions(_joint_group_positions))
      return true;
  }
  return false;
}

bool BorgArmController::moveByPose(
  geometry_msgs::Pose _pose) {
  bool success = {false};
  double secs = ros::Time::now().toSec() + MAX_ALLOWED_EXECUTION_TIME;
  std::vector<double> joint_group_positions;

  move_group_.setStartStateToCurrentState();
  move_group_.setPoseTarget(_pose);
  // move_group_.setGoalTolerance(GOAL_TOLERANCE);
  move_group_.setGoalPositionTolerance(GOAL_POSITION_TOLERANCE);
  move_group_.setGoalOrientationTolerance(GOAL_ORIENTATION_TOLERANCE);
  move_group_.setPlanningTime(MAX_ALLOWED_PLANNING_TIME);

  success = (move_group_.plan(plan_) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  joint_group_positions = plan_.trajectory_.joint_trajectory.points[
    plan_.trajectory_.joint_trajectory.points.size()-1].positions;

  if (success) {
    // move_group_.execute(plan_);
    while (secs > ros::Time::now().toSec()) {
      if (checkJointPositions(joint_group_positions))
        return true;
    }
  }
  return false;
}

bool BorgArmController::moveByPoseNoTrajectory(
  geometry_msgs::Pose _pose) {
  bool success = {false};
  uint attempts = 10;
  double timeout = 1;
  double secs = ros::Time::now().toSec() + MAX_ALLOWED_EXECUTION_TIME;
  std::vector<double> joint_group_positions;
  std::vector<double> old_joint_group_positions;


  current_state_ptr_ = move_group_.getCurrentState();

  current_state_ptr_->copyJointGroupPositions(
    joint_model_group_, joint_group_positions);
  // current_state_ptr_->enforceBounds();

  old_joint_group_positions = joint_group_positions;

  _pose.position.x += offset_x_;
  _pose.position.y += offset_y_;
  _pose.position.z += offset_z_;

  success = current_state_ptr_->setFromIK(
    joint_model_group_, _pose, attempts, timeout);

  if (success) {
    current_state_ptr_->copyJointGroupPositions(
      joint_model_group_, joint_group_positions);

    for (size_t i = 0; i < joint_group_positions.size(); i++) {
      ROS_INFO_STREAM("IK joint " << i << " old value: " <<
         old_joint_group_positions[i] << " new values: " <<
         joint_group_positions[i]);
    }

    for (size_t i = 0; i < 3; i++) {
      if (moveByJointAngleNoPlan(joint_group_positions))
        return true;
    }
  }
  ROS_WARN("IK Failed.");
  return false;
}



bool BorgArmController::checkJointPositions(
  std::vector<double> _joint_values) {
  double error = {0.0};
  std::vector<double> current_joint_values =
    move_group_.getCurrentJointValues();

  for (size_t i = 0; i < current_joint_values.size(); i++) {
    error += pow(_joint_values[i] - current_joint_values[i], 2);
  }
  error = sqrt(error);

  // ROS_INFO_STREAM("Move Group Joint error: " << error);
  if (error < MAX_ALLOWED_JOINT_ERROR)
    return true;
  else
    return false;
}

void BorgArmController::copyTf2PoseMsg(
  tf::StampedTransform* _tf, geometry_msgs::Pose* _pose) {
  _pose->orientation.x = _tf->getRotation().x();
  _pose->orientation.y = _tf->getRotation().y();
  _pose->orientation.z = _tf->getRotation().z();
  _pose->orientation.w = _tf->getRotation().w();
  _pose->position.x    = _tf->getOrigin().x();
  _pose->position.y    = _tf->getOrigin().y();
  _pose->position.z    = _tf->getOrigin().z();
}

bool BorgArmController::findTag() {
  for (size_t i = 0; i < tf_buttons_.size(); i++) {
    try {
      ros::Time now = ros::Time::now();
      listener_.waitForTransform("/world", tf_id_buttons_[i],
                                now, ros::Duration(3));
      listener_.lookupTransform("/world", tf_id_buttons_[i],
                                now, tf_buttons_[i]);

      copyTf2PoseMsg(&tf_buttons_[i], &pose_buttons_[i]);

      // ss_ << tf_id_buttons_[i] << " (x="
      //     << pose_buttons_[i].position.x << ", y="
      //     << pose_buttons_[i].position.y << ", z="
      //     << pose_buttons_[i].position.z << "); (x="
      //     << pose_buttons_[i].orientation.x << ", y="
      //     << pose_buttons_[i].orientation.y << ", z="
      //     << pose_buttons_[i].orientation.z << ", w="
      //     << pose_buttons_[i].orientation.w << ")" << std::endl;
      // ROS_INFO_STREAM(ss_.str());
    } catch (tf::TransformException ex) {
      // ROS_ERROR("%s", ex.what());
      ROS_INFO("Looking for tag: Did not find transforms for buttons!");
      return false;
    }
  }

  for (size_t i = 0; i < tf_boxes_.size(); i++) {
    try {
      ros::Time now = ros::Time::now();
      listener_.waitForTransform("/world", tf_id_boxes_[i],
                                now, ros::Duration(3));
      listener_.lookupTransform("/world", tf_id_boxes_[i],
                                now, tf_boxes_[i]);

      copyTf2PoseMsg(&tf_boxes_[i], &pose_boxes_[i]);

      // ss_ << tf_id_boxes_[i] << " (x="
      //     << pose_boxes_[i].position.x << ", y="
      //     << pose_boxes_[i].position.y << ", z="
      //     << pose_boxes_[i].position.z << "); (x="
      //     << pose_boxes_[i].orientation.x << ", y="
      //     << pose_boxes_[i].orientation.y << ", z="
      //     << pose_boxes_[i].orientation.z << ", w="
      //     << pose_boxes_[i].orientation.w << ")" << std::endl;
      // ROS_INFO_STREAM(ss_.str());
    } catch (tf::TransformException ex) {
      // ROS_ERROR("%s", ex.what());
      ROS_INFO("Looking for tag: Did not find transforms for boxes!");
      return false;
    }
  }

  try {
      tf::StampedTransform tf_aruco;
      ros::Time now = ros::Time::now();
      listener_.waitForTransform("/world", "/aruco_id4",
                                now, ros::Duration(3));
      listener_.lookupTransform("/world", "/aruco_id4",
                                now, tf_aruco);

      double roll, pitch, yaw;
      tf::Quaternion quat;
      quat = tf_aruco.getRotation();
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      roll  = toDeg(roll);
      pitch = toDeg(pitch);
      yaw   = toDeg(yaw);


      if (fabs(roll) <= 5 && fabs(pitch) <= 5) {
        box_orientation_ = 0;
      } else if (fabs(roll - 94) <= 5 && fabs(pitch) <= 5) {
        box_orientation_ = 1;
      } else {
        box_orientation_ = 2;
      }

      // ROS_INFO_STREAM("box orientation: " << box_orientation_);

      // ss_ << "box orientation: " << box_orientation_
      //     << " tf_aruco" << " (x="
      //     << tf_aruco.getOrigin().x() << ", y="
      //     << tf_aruco.getOrigin().y() << ", z="
      //     << tf_aruco.getOrigin().z() << "); (x="
      //     << roll << ", y="
      //     << pitch << ", z="
      //     << yaw << ")" << std::endl;
      // ROS_INFO_STREAM(ss_.str());
    } catch (tf::TransformException ex) {
      // ROS_ERROR("%s", ex.what());
      ROS_INFO("Looking for tag: Did not find transforms for boxes!");
      return false;
    }



  return true;
}

bool BorgArmController::stateMachine() {
  static int count = {0};
  char c;
  // ROS_INFO_STREAM("State: " << state_ << " Count: " << count);
  geometry_msgs::Pose pose, pose_old;
  geometry_msgs::PoseStamped pose_stamped;
  static int state_bkp = 0;

  switch (state_) {
    case AT_HOME:
      ROS_INFO("Moving to home position.");
      planning_scene_interface_.removeCollisionObjects(object_id_boxes_);
      planning_scene_interface_.addCollisionObjects(
        collision_objects_workbench_);

      moveByJointAngle("home");
      state_ = WAIT_START;

      break;

    case WAIT_START:
      if (start_scan_) {
        ROS_INFO("Manipulator will start! Measuring time!");
        time_begin_ = ros::Time::now();

        pose_stamped = move_group_.getCurrentPose("eef");
        pose = pose_stamped.pose;

        ROS_INFO_STREAM("CurrentPose START: " << pose);

        // moveByJointAngle("northwest");
        // moveByJointAngle("southwest");


        // // set desired pose
        // // Target (-0.320881, 0.17752, 1.00984) Orientation (-0.0180023, 0.0218943, 0.0264231, 0.999249)
        // pose.position.x = -0.313;
        // pose.position.y = 0.184;
        // pose.position.z = 0.984;
        // pose.orientation.x = 0;
        // pose.orientation.y = 0;
        // pose.orientation.z = 0;
        // pose.orientation.w = 1;

        // ROS_INFO_STREAM("DesiredPose: "  << pose);
        // pose_old = pose;

        // moveByPose(pose);

        // pose_stamped = move_group_.getCurrentPose("eef");
        // pose = pose_stamped.pose;
        // ROS_INFO_STREAM("CurrentPose END: " << pose);


        // pose_old.position.x -= pose.position.x;
        // pose_old.position.y -= pose.position.y;
        // pose_old.position.z -= pose.position.z;
        // pose_old.orientation.x -= pose.orientation.x;
        // pose_old.orientation.y -= pose.orientation.y;
        // pose_old.orientation.z -= pose.orientation.z;
        // pose_old.orientation.w -= pose.orientation.w;


        // ROS_INFO_STREAM("PoseDifference: " << pose_old);

        // start_scan_ = false;
        // while(!start_scan_);

        // moveByJointAngle("southwest");
        // moveByJointAngle("northwest");
        // moveByJointAngle("home");


        // sleep(60);
        // start_scan_ = false;
        // state_ = AT_HOME;

        state_ = AT_NORTH;
        // return 0;
      }
      break;


    case AT_NORTH:
      ROS_INFO("Moving to north position.");
      moveByJointAngle("north");
      sleep(SLEEP_AT_STATE);
      if (findTag()) {
        time_scan_ = ros::Time::now();
        putCollisionBoxes();
        offset_x_ = 0.0;
        offset_y_ = 0.0;
        offset_z_ = 0.0;
        state_ = AT_NEAR_BUTTON;
        state_bkp = AT_NORTHWEST;
      } else {
        state_ = AT_NORTHWEST;
      }
      break;

    case AT_NORTHWEST:
      ROS_INFO("Moving to northwest position.");
      moveByJointAngle("northwest");
      // sleep(SLEEP_AT_STATE);
      // if (findTag()) {
        time_scan_ = ros::Time::now();
      //   putCollisionBoxes();
      //   offset_x_ = 0.0;
      //   offset_y_ = 0.0;
      //   offset_z_ = 0.0;
      //   state_bkp = AT_SOUTHWEST;
      //   state_ = AT_NEAR_BUTTON;
      // } else {
      //   state_ = AT_SOUTHWEST;
      // }
      state_ = AT_SOUTHWEST;
      break;

    case AT_SOUTHWEST:
      ROS_INFO("Moving to southwest position.");
      moveByJointAngle("southwest");
      sleep(SLEEP_AT_STATE);
      if (findTag()) {
        time_scan_ = ros::Time::now();
        putCollisionBoxes();
        offset_x_ = 0.0;
        offset_y_ = 0.0;
        offset_z_ = 0.0;
        state_bkp = AT_NORTHWEST2;
        state_ = AT_NEAR_BUTTON;
      } else {
        state_ = AT_NORTHWEST2;
      }
      break;

    case AT_NORTHWEST2:
      ROS_INFO("Moving to northwest2 position.");
      moveByJointAngle("northwest2");
      sleep(SLEEP_AT_STATE);
      if (findTag()) {
        time_scan_ = ros::Time::now();
        putCollisionBoxes();
        offset_x_ = 0.0;
        offset_y_ = 0.0;
        offset_z_ = 0.0;
        state_bkp = AT_NORTH2;
        state_ = AT_NEAR_BUTTON;
      } else {
        state_ = AT_NORTH2;
      }
      break;

    case AT_NORTH2:
      ROS_INFO("Moving to north2 position.");
      moveByJointAngle("north2");
      sleep(SLEEP_AT_STATE);
      if (findTag()) {
        time_scan_ = ros::Time::now();
        putCollisionBoxes();
        offset_x_ = 0.0;
        offset_y_ = 0.0;
        offset_z_ = 0.0;
        state_bkp = AT_NORTHEAST;
        state_ = AT_NEAR_BUTTON;
      } else {
        state_ = AT_NORTHEAST;
      }
      break;

    case AT_NORTHEAST:
      ROS_INFO("Moving to northeast position.");
      moveByJointAngle("northeast");
      // if (findTag()) {
        time_scan_ = ros::Time::now();
      //   putCollisionBoxes();
      //   state_bkp = AT_SOUTHEAST;
      //   state_ = AT_NEAR_BUTTON;
      // } else {
        state_ = AT_SOUTHEAST;
      // }
      break;

    case AT_SOUTHEAST:
      ROS_INFO("Moving to southeast position.");
      moveByJointAngle("southeast");
      sleep(SLEEP_AT_STATE);
      if (findTag()) {
        time_scan_ = ros::Time::now();
        putCollisionBoxes();
        offset_x_ = 0.0;
        offset_y_ = 0.0;
        offset_z_ = 0.0;
        state_bkp = TAG_NOT_FOUND;
        state_ = AT_NEAR_BUTTON;
      } else {
        state_ = TAG_NOT_FOUND;
      }
      break;

    case TAG_NOT_FOUND:
      ROS_INFO("Tag NOT found, returning home.");
      moveByJointAngle("northeast2");
      state_ = AT_HOME;
      break;

    case AT_NEAR_BUTTON:
      ROS_INFO("Tag found. Moving near button.");
      if (moveByPose(pose_buttons_[0])) {
        state_ = AT_BUTTON;
        count = 0;
      } else {
        if (count >= 2) {
          count = 0;
          state_ = state_bkp;
        } else {
          count++;
          state_ = AT_NEAR_BUTTON;
        }
      }
      break;

    case AT_BUTTON:
      ROS_INFO("Pressing button.");
      // pose_buttons_[1].position.y = 0.249401;
      pose_target_ = pose_buttons_[1];
      pose_target_.position.x += offset_x_;
      pose_target_.position.y += offset_y_;
      pose_target_.position.z += offset_z_;

      if (moveByPoseNoTrajectory(pose_buttons_[1])) {
        offset_x_ = 0.0;
        offset_y_ = 0.0;
        offset_z_ = 0.0;
        pose_stamped = move_group_.getCurrentPose("eef");
        pose_achieved_ = pose_stamped.pose;
        time_button_ = ros::Time::now();
        state_ = AT_LEAVING_BUTTON;
        count = 0;
      } else {
        if (count >= 2) {
          count = 0;
          state_ = AT_HOME;
        } else {
          count++;
          state_ = AT_BUTTON;
        }
      }
      break;

    case AT_LEAVING_BUTTON:
      ROS_INFO("Tag found. Leaving button.");


      if (moveByPoseNoTrajectory(pose_buttons_[0])) {
        offset_x_ = 0.0;
        offset_y_ = 0.0;
        offset_z_ = 0.0;
        state_ = AT_HOME_AND_END;
        count = 0;
      } else {
        if (count >= 2) {
          count = 0;
          state_ = AT_HOME;
        } else {
          count++;
          state_ = AT_LEAVING_BUTTON;
        }
      }
      break;

    case AT_HOME_AND_END:
      moveByJointAngle("home");
      time_end_ = ros::Time::now();

      moveByJointAngle("rest");

      ss_.clear();
      ss_ << "Time scan: " << time_scan_.toSec() - time_begin_.toSec()
          << " Time button: " << time_button_.toSec() - time_begin_.toSec()
          << " Time end: " << time_end_.toSec() - time_begin_.toSec()
          << " Target ("
          << pose_target_.position.x << ", "
          << pose_target_.position.y << ", "
          << pose_target_.position.z << ") Orientation ("
          << pose_target_.orientation.x << ", "
          << pose_target_.orientation.y << ", "
          << pose_target_.orientation.z << ", "
          << pose_target_.orientation.w << ")."
          << " Achieved ("
          << pose_achieved_.position.x << ", "
          << pose_achieved_.position.y << ", "
          << pose_achieved_.position.z << ") Orientation ("
          << pose_achieved_.orientation.x << ", "
          << pose_achieved_.orientation.y << ", "
          << pose_achieved_.orientation.z << ", "
          << pose_achieved_.orientation.w << ")." << std::endl;
      out_file_ << ss_.str();
      ROS_INFO_STREAM(ss_.str());
      return 0;

    default:
      state_ = AT_HOME;
      break;
  }
  return 1;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "BorgArmController");

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);

  spinner.start();

  BorgArmController move_group_interface(&nh);
  while (ros::ok()) {
    if (!move_group_interface.stateMachine())
      break;
  }

  return 0;
}