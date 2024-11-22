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
#include <boost/asio.hpp>  // Include boost asio

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
  : Node("pose_action_client", node_options), 
  goal_done_(false),
  io_context_(),
  acceptor_(io_context_),  // Initialize acceptor
  socket_(io_context_)  // Initialize socket
  {
    this->client_ptr_ = rclcpp_action::create_client<NavToPose>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "navigate_to_pose");

    // this->timer_ = this->create_wall_timer(
    //   std::chrono::milliseconds(2000),
    //   std::bind(&PoseActionClient::send_goal, this));

    this->subscription_clock_ = create_subscription<rosgraph_msgs::msg::Clock>("clock", rclcpp::ClockQoS(),
                std::bind(&PoseActionClient::topic_callback_clock, this, std::placeholders::_1));


    // Start TCP server to accept incoming requests
    start_tcp_server();
  }


  ~PoseActionClient()
  {
      io_context_.stop();
      if (io_context_thread_ && io_context_thread_->joinable()) {
          io_context_thread_->join();
      }
  }

  bool is_goal_done() const
  {
    return this->goal_done_;
  }

  void send_goal(double x, double y, double w)
  {
    using namespace std::placeholders;

    // this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
      return;
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

    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;

    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = 1.0;
    goal_msg.pose.pose.orientation.w = w;

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
  std::unique_ptr<std::thread> io_context_thread_;


  // TCP server components
  boost::asio::io_context io_context_;
  boost::asio::ip::tcp::acceptor acceptor_;
  boost::asio::ip::tcp::socket socket_;

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


  void start_tcp_server()
  {
    using boost::asio::ip::tcp;
    tcp::endpoint endpoint(tcp::v4(), 12345);  // Listening on port 12345
    acceptor_.open(tcp::v4());
    acceptor_.bind(endpoint);
    acceptor_.listen();

    accept_connection();


    // Log server details
    auto endpoint_my = acceptor_.local_endpoint();
    RCLCPP_INFO(this->get_logger(), "TCP Server started on IP: %s, Port: %d",
                endpoint_my.address().to_string().c_str(), endpoint_my.port());
  }

  void accept_connection()
  {
    acceptor_.async_accept(socket_, [this](boost::system::error_code ec) {
      if (!ec) {
        // Once a connection is accepted, start reading data
        RCLCPP_INFO(this->get_logger(), "TCP Connection established");
        handle_client_request();
      }
    });
    
    // Run io_context in a background thread to process async events
    if (!io_context_thread_) {
        io_context_thread_ = std::make_unique<std::thread>([this]() {
            io_context_.run();
        });
    }
  }

  void handle_client_request()
  {
      try {
          // Buffer to store incoming data
          std::array<char, 128> buffer;
          boost::system::error_code ec;

          // Read data from the socket
          size_t length = socket_.read_some(boost::asio::buffer(buffer), ec);
          if (!ec) {
              std::string request(buffer.data(), length);
              RCLCPP_INFO(this->get_logger(), "Received request: %s", request.c_str());

              // Parse the request (e.g., assume it is a comma-separated format: "x,y,z")
              std::stringstream ss(request);
              std::string x_str, y_str, z_str;
              std::getline(ss, x_str, ',');
              std::getline(ss, y_str, ',');
              std::getline(ss, z_str, ',');

              // Process or send the goal
              float x = std::stof(x_str);
              float y = std::stof(y_str);
              float z = std::stof(z_str);
              RCLCPP_INFO(this->get_logger(), "Parsed coordinates: x=%f, y=%f, z=%f", x, y, z);

              send_goal(x,y,z);

              // Example acknowledgment
              std::string ack_message = "Request processed successfully.\n";
              boost::asio::write(socket_, boost::asio::buffer(ack_message));
          } else {
              RCLCPP_ERROR(this->get_logger(), "Error reading from socket: %s", ec.message().c_str());
          }

          // Close the current socket after handling request
          socket_.close();

          // Prepare to accept the next connection
          accept_connection();

      } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "Exception in handle_client_request: %s", e.what());
          socket_.close();
          accept_connection();  // Ensure the server continues to accept new connections
      }
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
