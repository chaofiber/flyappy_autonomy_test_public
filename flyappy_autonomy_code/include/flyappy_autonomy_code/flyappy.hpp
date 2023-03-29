#pragma once
#include <vector>
#include <iostream>

// define data structure

struct Position
{
    double x;
    double y;
};

struct Velocity
{
    double x;
    double y;
};

struct Acceleration
{
    double x;
    double y;
};

struct LaserScan
{
    int size;
    std::vector<double> ranges;
    std::vector<double> intensities;
    std::vector<double> angles;
    double angle_min;
    double angle_max;
    double angle_increment;
    double range_min;
    double range_max;
};

enum FlyingMode : int
{
    kCruise = 0,
    kSearching = 1,
    kSearchingDown = 2,
    kApproaching = 3,
    kBangBang = 4,
};

struct BangBangPlan
{
    double X;
    double Y;
    double Vx_max;
    double Vy_max;
    double progress_x;
    double progress_y;
};

class Flyappy
{
  public:
    Flyappy();
    // according to the game, one strategy is to make sure that the bird always flys
    // throught the asteriods at the same height. It is safer to fly through, and for
    // locating the next through point correctl? not sure

    // rank all angles

    // according to the image size, width is 430 pixel = 4.3 meters, which means each line
    // of asteriods is about 2.1 meters apart. the height is 480 pixesl = 4.8meters, the
    // minimum gap between is about 0.8 meters for the bird to fly through. some functions
    // that may be useful:
    void setDesiredVelocity(Velocity& des_vel) { des_vel_ = des_vel; }
    void updateVelocity(Velocity& curr_vel) { curr_vel_ = curr_vel; }
    void updateLaserScan(LaserScan& laser_scan);
    Velocity getCurrVelocity() { return curr_vel_; }
    Velocity getDesiredVelocity() { return des_vel_; }

    bool estimateDistanceToNextAsteriod();
    void estimateDistanceInY();
    bool checkIfFree(int index);
    bool searchForConsistentFreeAngle();

    Acceleration getAccelerationCommand();
    void updateBangBangPlan();
    bool updateSteeringAngle();
    void updateMode();

    int getMode() { return mode_; }

    void printLaserscan(LaserScan& laser_scan) {
      for (int i = 0; i < laser_scan.size; i++) {
        std::cout << laser_scan.intensities[i] << " ";
      }
    }

  private:
    // some variables that may be useful:
    Velocity curr_vel_;
    LaserScan curr_laser_scan_;
    Velocity des_vel_;
    double safe_range_;  // make sure all the range value larger than the safe_range

    std::vector<std::vector<int>> ranking_;  // store the ranking of the angles
    int history_capacity_;                   // store number of history buffer
    std::vector<std::vector<double>>
            laser_range_history_;  // store the history of laser range
    std::vector<std::vector<int>>
            laser_intensity_history_;  // store the history of laser intensity

    // maximum acceleration
    double max_acceleration_x_ = 3;
    double max_acceleration_y_ = 35;
    double update_dt_ = 0.033333;  // 30Hz

    // estimated
    double distance_to_next_asteriod_ =
            0;  // estimated mean distance to the next column of asteriod
    std::vector<double>
            relative_distance_x_;  // estimated relative distance in x direction
    std::vector<double>
            relative_distance_y_;        // estimated relative distance in y direction
    std::vector<double> distance_in_y_;  // estimated relative distance in y direction
    double safe_gap_ = 0.8;

    double threshold_ = 0.4;  // threshold for checking a laser is blocked by the wall
                              // instead of asteriods

    double steering_angle_ = 100;  // steering angle

    int mode_ = kCruise;
    BangBangPlan bang_bang_plan_;
    Position target_relative_pos_;
    double approaching_speed_ = 0.2;
    double cruise_speed_ = 1.0;

    double searching_speed_y_ = 0.5;

    std::vector<std::vector<double>> range_buffer_;
    int range_buffer_size_ = 6;

    // search related
    bool search_started_ = false;
    double search_start_time_ = 0;
    double search_duration_ = 0.5;
    double search_timer_ = 0.0;
    bool found_target_ = false;
};
