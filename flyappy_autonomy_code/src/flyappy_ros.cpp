#include "flyappy_autonomy_code/flyappy_ros.hpp"

constexpr uint32_t QUEUE_SIZE = 5u;

FlyappyRos::FlyappyRos(ros::NodeHandle& nh)
    : pub_acc_cmd_(nh.advertise<geometry_msgs::Vector3>("/flyappy_acc", QUEUE_SIZE)),
      sub_vel_(nh.subscribe("/flyappy_vel", QUEUE_SIZE, &FlyappyRos::velocityCallback,
                            this)),
      sub_laser_scan_(nh.subscribe("/flyappy_laser_scan", QUEUE_SIZE,
                                   &FlyappyRos::laserScanCallback, this)),
      sub_game_ended_(nh.subscribe("/flyappy_game_ended", QUEUE_SIZE,
                                   &FlyappyRos::gameEndedCallback, this))
{
}

void FlyappyRos::velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    // update velocity
    curr_vel_.x = msg->x;
    curr_vel_.y = msg->y;
    flyappy_.updateVelocity(curr_vel_);


    std::cout << "Mode: " << flyappy_.getMode() << std::endl;

    // update mode
    flyappy_.updateMode();


    // obtain acceleration command
    Acceleration cmd = flyappy_.getAccelerationCommand();

    // Example of publishing acceleration command to Flyappy
    geometry_msgs::Vector3 acc_cmd;

    acc_cmd.x = cmd.x;
    acc_cmd.y = cmd.y;
    pub_acc_cmd_.publish(acc_cmd);
}

void FlyappyRos::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // Example of printing laser angle and range
    // ROS_INFO("Laser range: %f, angle: %f", msg->ranges[0], msg->angle_min);
    laser_scan_.angle_min = msg->angle_min;
    laser_scan_.angle_max = msg->angle_max;
    laser_scan_.range_min = msg->range_min;
    laser_scan_.range_max = msg->range_max;
    laser_scan_.angle_increment = msg->angle_increment;
    laser_scan_.ranges.resize(msg->ranges.size());
    laser_scan_.ranges.clear();
    laser_scan_.intensities.resize(msg->intensities.size());
    laser_scan_.intensities.clear();
    for (const auto& value : msg->ranges)
    {
        laser_scan_.ranges.push_back(static_cast<double>(value));
    }
    for (const auto& value : msg->intensities)
    {   
        laser_scan_.intensities.push_back(static_cast<double>(value));
    }
    laser_scan_.size = msg->ranges.size();
    std::vector<double> angles(laser_scan_.size, 0);
    for (int i = 0; i < laser_scan_.size; i++)
    {
        angles[i] = laser_scan_.angle_min + i * laser_scan_.angle_increment;
    }
    laser_scan_.angles = angles;
    flyappy_.updateLaserScan(laser_scan_);
}

void FlyappyRos::gameEndedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
        ROS_INFO("Crash detected.");
    }
    else
    {
        ROS_INFO("End of countdown.");
    }

    flyappy_ = {};
}
