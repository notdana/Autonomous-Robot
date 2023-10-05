/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <thread>

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.05;
}

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
  , same_goal_count_(0)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  progress_timeout_ = timeout;
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size);

  // DANA ADDED THIS
  home_publisher_ = this->create_publisher<std_msgs::msg::Empty>("go_home", 10);
  map_end_publisher_ = this->create_publisher<std_msgs::msg::String>("mapState", 10);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
    current_goal_marker_publisher_ = this->create_publisher<
        visualization_msgs::msg::Marker>("explore/current_goal", 10);
    blacklisted_frontier_marker_publisher_ = this->create_publisher<
        visualization_msgs::msg::Marker>("explore/blacklisted_frontier", 10);
  }

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  // Start exploration right away
  exploring_timer_->execute_callback();
}

Explore::~Explore()
{
  stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::msg::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::msg::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::msg::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  RCLCPP_INFO(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
#ifdef ELOQUENT
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#elif DASHING
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#else
  m.lifetime = rclcpp::Duration::from_seconds(0);  // foxy onwards
#endif
  // m.lifetime = rclcpp::Duration::from_nanoseconds(0); // suggested in
  // galactic
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    // m.pose.position = {}; // compile warning
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  RCLCPP_INFO(logger_, "------------------------------------------");
  if(finished_exploring_){
    RCLCPP_INFO(logger_, "Exploration finished, not making new plan");
    exploring_timer_->cancel();
    return;
  }
  RCLCPP_INFO(logger_, "Making plan callback");

  if(!first_exploration_cycle){
    RCLCPP_INFO(logger_, "Sending first goal to exploration goal");

    last_progress_ = this->now();
    geometry_msgs::msg::Point target_position;
    target_position.x = initial_pose_.position.x + 1.0;
    target_position.y = initial_pose_.position.y + 1.0;
    // send goal to move_base if we have something new to pursue
    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.pose.pose.position = target_position;
    goal.pose.pose.orientation.w = 1.;
    goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
    goal.pose.header.stamp = this->now();

    auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        [this,
        target_position](const NavigationGoalHandle::WrappedResult& result) {
          reachedGoal(result, target_position);
        };
    move_base_client_->async_send_goal(goal, send_goal_options);

    first_exploration_cycle = true;
    return;
  }

  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  RCLCPP_INFO(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    RCLCPP_WARN(logger_, "No frontiers found, stopping.");
    stop(true);
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);

    visualization_msgs::msg::Marker blacklisted_frontier;
    blacklisted_frontier.header.frame_id = costmap_client_.getGlobalFrameID();
    blacklisted_frontier.header.stamp = this->now();
    blacklisted_frontier.ns = "goal";
    blacklisted_frontier.id = 0;
    blacklisted_frontier.type = visualization_msgs::msg::Marker::POINTS;
    blacklisted_frontier.action = visualization_msgs::msg::Marker::ADD;
    blacklisted_frontier.points = frontier_blacklist_;
    blacklisted_frontier.scale.x = 0.5;
    blacklisted_frontier.scale.y = 0.5;
    blacklisted_frontier.scale.z = 0.5;
    blacklisted_frontier.color.r = 255;
    blacklisted_frontier.color.b = 255;
    blacklisted_frontier.color.a = 255;
    blacklisted_frontier_marker_publisher_->publish(blacklisted_frontier);
  }

  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });
  if (frontier == frontiers.end()) {
    RCLCPP_WARN(logger_, "All frontiers traversed/tried out, stopping.");
    stop(true);
    return;
  }
  geometry_msgs::msg::Point target_position = frontier->centroid;

  // time out if we are not making any progress
  bool same_goal = same_point(prev_goal_, target_position);

  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ > frontier->min_distance) {
    RCLCPP_INFO(logger_, "making progress");
    // we have different goal or we made some progress
    last_progress_ = this->now();
    prev_distance_ = frontier->min_distance;
  }
  // black list if we've made no progress for a long time
  if ((this->now() - last_progress_ >
      tf2::durationFromSec(progress_timeout_)) && !resuming_) {
    frontier_blacklist_.push_back(target_position);
    RCLCPP_INFO(logger_, "Adding current goal to black list due to timeout");
    makePlan();
    return;
  }
  else{
    RCLCPP_INFO_STREAM(logger_, "sec from last_progress: "<< (this->now() - last_progress_).seconds());
    // std::cout<< (this-now() - last_progress_).seconds() <<std::endl;
    std::cout<<this->now().nanoseconds()<<std::endl;
    std::cout<<last_progress_.nanoseconds()<<std::endl;


  }

  // ensure only first call of makePlan was set resuming to true
  if (resuming_) {
    resuming_ = false;
  }

  visualization_msgs::msg::Marker goal_marker;
  goal_marker.header.frame_id = costmap_client_.getGlobalFrameID();
  goal_marker.header.stamp = this->now();
  goal_marker.ns = "goal";
  goal_marker.id = 0;
  goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
  goal_marker.action = visualization_msgs::msg::Marker::ADD;
  goal_marker.pose.position = target_position;
  goal_marker.pose.orientation.w = 1.0;
  goal_marker.scale.x = 0.5;
  goal_marker.scale.y = 0.5;
  goal_marker.scale.z = 0.5;
  goal_marker.color.r = 255;
  goal_marker.color.g = 255;
  goal_marker.color.a = 255;
  current_goal_marker_publisher_->publish(goal_marker);

  // we don't need to do anything if we still pursuing the same goal
  if (same_goal){
    RCLCPP_INFO(logger_, "Same goal, not making new plan");
    return;
  }

  last_progress_ = this->now();
  RCLCPP_INFO(logger_, "Sending goal to move base nav2");

  // send goal to move_base if we have something new to pursue
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;
  goal.pose.pose.orientation.w = 1.;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  // send_goal_options.goal_response_callback =
  // std::bind(&Explore::goal_response_callback, this, _1);
  // send_goal_options.feedback_callback =
  //   std::bind(&Explore::feedback_callback, this, _1, _2);
  send_goal_options.feedback_callback =
    [this,
      target_position](typename rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
        feedbackSendGoal(feedback, target_position);
      };
  send_goal_options.result_callback = 
      [this,
       target_position](const NavigationGoalHandle::WrappedResult& result) {
        reachedGoal(result, target_position);
      };
  found_new_fountier = true;
  RCLCPP_INFO(logger_, "Sending new goal to move base nav2");
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");

  // DANA ADDED THIS
  std_msgs::msg::Empty empty_msg;
  home_publisher_->publish(empty_msg);

  std_msgs::msg::String map_msg;
  map_msg.data = "finished mapping";
  map_end_publisher_->publish(map_msg);

  // auto goal = nav2_msgs::action::NavigateToPose::Goal();
  // goal.pose.pose.position = initial_pose_.position;
  // goal.pose.pose.orientation = initial_pose_.orientation;
  // goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  // goal.pose.header.stamp = this->now();

  // auto send_goal_options =
  //     rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  // auto initial_position = initial_pose_.position;
  // send_goal_options.result_callback =
  //     [this, initial_position](const NavigationGoalHandle::WrappedResult& result) {
  //       reachedGoal(result, initial_position);
  //     };
  // move_base_client_->async_send_goal(goal, send_goal_options);
}

bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  constexpr static size_t tolerace = 5;
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);
    double dist = x_diff * x_diff + y_diff * y_diff;
    // costmap2d->getResolution() = 0.05
    // tolerace * costmap2d->getResolution() = 0.25
    // if (x_diff < tolerace * costmap2d->getResolution() &&
    //     y_diff < tolerace * costmap2d->getResolution())
    if(dist < 1.0*1.0)
      return true;
  }
  return false;
}

void Explore::feedbackSendGoal(const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback,
                                     const geometry_msgs::msg::Point& frontier_goal)

{
  if(feedback->number_of_recoveries >=8){
    RCLCPP_INFO(logger_, "Number of recoveries is greater than 8, cancel the goal and add it to the blacklist");
    // FIXME: should abort instead of cancel
    move_base_client_->async_cancel_all_goals();
  }
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      found_new_fountier = false;
      RCLCPP_INFO(logger_, "Goal was successful");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(logger_, "Goal was aborted");
      if(found_new_fountier){
        // reset the found_new_fountier flag
        found_new_fountier = false;
      }
      else{
        frontier_blacklist_.push_back(frontier_goal);
        RCLCPP_INFO(logger_, "Adding current goal to black list");  
      }
      // If it was aborted probably because we've found another frontier goal,
      // so just return and don't make plan again
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(logger_, "Goal was canceled");
      found_new_fountier = false;
      frontier_blacklist_.push_back(frontier_goal);
      RCLCPP_INFO(logger_, "Adding current goal to black list");  
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }
  // find new goal immediately regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  // oneshot_ = relative_nh_.createTimer(
  //     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
  //     true);

  // Because of the 1-thread-executor nature of ros2 I think timer is not
  // needed.
  // (Alex) remove it because it will cause the timer to be restarted
  makePlan();
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started.");
}

void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "Exploration stopped.");
  RCLCPP_INFO(logger_, "async_cancel_all_goals() is called from stop()");
  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();
  
  if (return_to_init_ && finished_exploring) {
    finished_exploring_ = true;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    returnToInitialPose();
  }
}

void Explore::resume()
{
  resuming_ = true;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  exploring_timer_->execute_callback();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
