#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>

class LaserScanFilter : public rclcpp::Node
{
public:
    LaserScanFilter() : Node("laser_scan_filter")
    {
        // Create subscriber for laser scan data
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LaserScanFilter::scan_callback, this, std::placeholders::_1));

        // Create publisher for filtered laser scan data
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/filtered_scan", 10);

        RCLCPP_INFO(this->get_logger(), "Laser scan filter node initialized");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        sensor_msgs::msg::LaserScan filtered_scan = *msg;

        // Calculate indices for 0 to 120 degrees
        // Assuming angle_min is -π and angle_max is π
        float angle_min_filtered = 0.0;
        float angle_max_filtered = 2.0944; // 120 degrees in radians

        int start_index = (angle_min_filtered - msg->angle_min) / msg->angle_increment;
        int end_index = (angle_max_filtered - msg->angle_min) / msg->angle_increment;

        // Resize ranges and intensities vectors
        filtered_scan.ranges.clear();
        filtered_scan.intensities.clear();

        // Copy only the data within our desired range
        for (int i = start_index; i <= end_index; i++)
        {
            if (i >= 0 && i < static_cast<int>(msg->ranges.size()))
            {
                filtered_scan.ranges.push_back(msg->ranges[i]);
                if (!msg->intensities.empty())
                {
                    filtered_scan.intensities.push_back(msg->intensities[i]);
                }
            }
        }

        // Update scan parameters
        filtered_scan.angle_min = angle_min_filtered;
        filtered_scan.angle_max = angle_max_filtered;
        filtered_scan.header.stamp = this->get_clock()->now();

        // Publish filtered scan
        publisher_->publish(filtered_scan);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScanFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
