#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <vector>
#include <cmath>

using std::placeholders::_1;

class SliderControl : public rclcpp::Node
{
public:
    SliderControl() : Node("slider_control")
    {
        sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "joint_commands", 10, std::bind(&SliderControl::sliderCallback, this, _1));

        arm_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "arm_controller/joint_trajectory", 10);
        gripper_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "gripper_controller/joint_trajectory", 10);

        current_positions_.resize(6, 0.0);
        target_positions_.resize(6, 0.0);

        smoothing_rate_ = 100.0;
        alpha_ = 0.1;

        timer_ = create_wall_timer(
            std::chrono::milliseconds(int(1000.0 / smoothing_rate_)),
            std::bind(&SliderControl::updateLoop, this));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> current_positions_;
    std::vector<double> target_positions_;
    double smoothing_rate_;
    double alpha_;

    void sliderCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < 6) return;
        for (size_t i = 0; i < 6; ++i)
            target_positions_[i] = msg->position[i];
    }

    void updateLoop()
    {
        bool changed = false;
        for (size_t i = 0; i < 6; ++i)
        {
            double diff = target_positions_[i] - current_positions_[i];
            if (std::fabs(diff) > 1e-4)
            {
                current_positions_[i] += diff * alpha_;
                changed = true;
            }
        }

        if (changed)
        {
            trajectory_msgs::msg::JointTrajectory arm_msg, gripper_msg;
            arm_msg.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5"};
            gripper_msg.joint_names = {"joint_6"};

            trajectory_msgs::msg::JointTrajectoryPoint arm_point, gripper_point;
            arm_point.positions = std::vector<double>(current_positions_.begin(), current_positions_.begin() + 5);
            gripper_point.positions = {current_positions_[5]};
            arm_point.time_from_start.sec = 0;
            gripper_point.time_from_start.sec = 0;

            arm_msg.points.push_back(arm_point);
            gripper_msg.points.push_back(gripper_point);

            arm_pub_->publish(arm_msg);
            gripper_pub_->publish(gripper_msg);
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SliderControl>());
    rclcpp::shutdown();
    return 0;
}
