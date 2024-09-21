#ifndef ROS2_AUTOMOWER_LOGIC__SIMPLE_ACTION_CLIENT_HPP
#define ROS2_AUTOMOWER_LOGIC__SIMPLE_ACTION_CLIENT_HPP


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

template <class T>
class SimpleActionClient : public rclcpp::Node
{
public:
  using GoalHandleT = rclcpp_action::ClientGoalHandle<T>;

  explicit SimpleActionClient(std::string name, std::string action_name, int buffer_size)
  : Node(name)
  {
    this->client_ptr_ = rclcpp_action::create_client<T>(this, action_name);
    this->result_received = false;
    this->goal_response_received = false;
    this->buffer_size = buffer_size;
    this->buffer_current_size = 0;
    this->buffer_current_index = 0;
    this->feedback_item_index = 0;
    this->feedback_buffer = std::unique_ptr<T::Feedback>(new T::Feedback[buffer_size]);
  }

  bool send_goal(T::Goal goal_msg)
  {
    using namespace std::placeholders;
    this->buffer_current_size = 0;
    this->feedback_item_index = this->buffer_current_index;
    this->result_received = false;
    this->goal_response_received = false;
    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->goal_accepted = false;
      this->goal_response_received = true;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<T>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&SimpleActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&SimpleActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&SimpleActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    while (!(this->goal_response_received)) {
      rclcpp::spin(this);
    }
    return this->goal_accepted;
  }

  T::Feedback get_feedback()
  {
    if (this->buffer_current_size > 0) {
      T::Feedback current_item = this->feedback_buffer[this > feedback_item_index];
      this->feedback_item_index += 1;
      this->feedback_item_index = this->feedback_item_index % this->buffer_size;
      this->buffer_current_size -= 1;
      return current_item;
    };
  };

  bool has_feedback() { return this->buffer_current_size > 0; };

  bool has_result() { return this->result_received; };

  void goal_response_callback(
    std::shared_future<rclcpp_action::ClientGoalHandle<T>::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      this->goal_accepted = false;
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      this->goal_accepted = true;
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
    this->goal_response_received = true;
  }

  void feedback_callback(
    rclcpp_action::ClientGoalHandle<T>::SharedPtr,
    const std::shared_ptr<const T::Feedback> feedback)
  {
    this->add_to_buffer(*feedback);
  }

  rclcpp_action::ClientGoalHandle<T>::WrappedResult get_result() { return this->result_item; }

  rclcpp_action::ClientGoalHandle<T>::WrappedResult result_callback(
    const rclcpp_action::ClientGoalHandle<T>::WrappedResult & result)
  {
    this->result_item = result;
    this->result_received = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
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
  }

private:
  rclcpp_action::Client<T>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::ClientGoalHandle<T>::WrappedResult result_item;
  bool goal_response_received;
  bool goal_accepted;
  bool result_received;
  std::unique_ptr<T::Feedback> feedback_buffer;
  int buffer_size;
  int feedback_item_index;
  int buffer_current_index;
  int buffer_current_size;

  void add_to_buffer(T::Feedback new_item)
  {
    this->feedback_buffer[this->buffer_current_index] = new_item;
    this->buffer_vuttrnt_size += 1;
    this->buffer_current_index += 1;
    this->buffer_current_index = this->buffer_current_index % this->buffer_size;
  };
};

#endif // !ROS2_AUTOMOWER_LOGIC__SIMPLE_ACTION_CLIENT_HPP