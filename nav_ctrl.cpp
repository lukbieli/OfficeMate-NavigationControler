// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cinttypes>
#include <functional>
#include <future>
#include <memory>
#include <string>

// #include "example_interfaces/action/fibonacci.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

class PoseActionClient : public rclcpp::Node
{
public:
  // using NavToPose = example_interfaces::action::NavToPose;
  using NavToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandlePose = rclcpp_action::ClientGoalHandle<NavToPose>;

  explicit PoseActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("pose_action_client", node_options), goal_done_(false)
  {
    this->client_ptr_ = rclcpp_action::create_client<NavToPose>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "navigate_to_pose");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2000),
      std::bind(&PoseActionClient::send_goal, this));

    this->subscription_clock_ = create_subscription<rosgraph_msgs::msg::Clock>("clock", rclcpp::ClockQoS(),
                std::bind(&PoseActionClient::topic_callback_clock, this, std::placeholders::_1));
  }

  bool is_goal_done() const
  {
    return this->goal_done_;
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    // if (this->timeOk_ == false)
    // {
    //   RCLCPP_ERROR(this->get_logger(), "Time not recieved after waiting");
    //   this->goal_done_ = true;
    //   return;
    // }


    auto goal_msg = NavToPose::Goal();
    // goal_msg.pose.header.stamp.sec = this->sec_;
    // goal_msg.pose.header.stamp.nanosec = this->nsec_;  

    goal_msg.pose.header.frame_id = "map";

    goal_msg.pose.pose.position.x = 0.0;
    goal_msg.pose.pose.position.y = 4.0;
    goal_msg.pose.pose.position.z = 0.0;

    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = 1.0;
    goal_msg.pose.pose.orientation.w = 0.5;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<NavToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&PoseActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&PoseActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&PoseActionClient::result_callback, this, _1);
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<NavToPose>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr subscription_clock_;
  int32_t sec_;
  uint32_t nsec_;
  bool timeOk_ = false;

  void goal_response_callback(GoalHandlePose::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandlePose::SharedPtr,
    const std::shared_ptr<const NavToPose::Feedback> feedback)
  {
    std::stringstream ss;
    ss << " Current position: " << this->poseToStr(feedback->current_pose.pose);
    ss << " Distance remaining: " << feedback->distance_remaining;

    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandlePose::WrappedResult & result)
  {
    this->goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received");
  }

  void topic_callback_clock(const rosgraph_msgs::msg::Clock &msg)
  {
      // RCLCPP_INFO_STREAM(this->get_logger(), "Sec: " << msg.clock.sec << " Nsec: " << msg.clock.nanosec);
      this->sec_ = msg.clock.sec;
      this->nsec_ = msg.clock.nanosec;
      this->timeOk_ = true;
  }

  std::string poseToStr(const geometry_msgs::msg::Pose pose)
  {
      std::string result = "";
      result += std::to_string(pose.position.x);
      result += ",";
      result += std::to_string(pose.position.y);
      result += ",";
      result += std::to_string(pose.position.z);

      return result;
  }
};  // class PoseActionClient

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<PoseActionClient>();

  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }

  rclcpp::shutdown();
  return 0;
}
