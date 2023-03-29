#include "flyappy_autonomy_code/flyappy.hpp"

#include <cmath>
#include <iostream>
#include <ostream>
#include <vector>

Flyappy::Flyappy() : mode_(kCruise) { curr_laser_scan_.size = 0; };

Acceleration Flyappy::getAccelerationCommand()
{
    Acceleration acc_cmd;
    if (mode_ == kSearching)
    {
        // search up first, and search down, keep x velocity equals zero
        acc_cmd.x = -max_acceleration_x_ * curr_vel_.x;  // try to keep vel_x=0
        acc_cmd.y = -2.0 * (curr_vel_.y - 0.5);
        if (search_timer_ > 0.2){
            acc_cmd.y = -4.0 * (curr_vel_.y + 0.5);
        }
    }
    if (mode_ == kApproaching)
    {
        // need some more finetuning here
        acc_cmd.x = -1.0 * (curr_vel_.x - 1.0);
        acc_cmd.y = -15 * (curr_vel_.y) - 5.0 * steering_angle_;  // try to keep vel_y=0
    }
    if (mode_ == kCruise)
    {
        acc_cmd.x = -1.0 * (curr_vel_.x - 1.0);
        acc_cmd.y = -curr_vel_.y;
    }
    if (mode_ == kBangBang)
    {
        acc_cmd.x = -1.0 * curr_vel_.x;  // try to keep vel_x=0
        // as starting point, just try to keep constant velocity
        if (bang_bang_plan_.progress_y < bang_bang_plan_.Y)
        {
            acc_cmd.y = -1.0 * (curr_vel_.y - 0.5);
            bang_bang_plan_.progress_y += curr_vel_.y * update_dt_;
        }
        else
        {
            acc_cmd.y = -1.0 * (curr_vel_.y + 0.5);
            bang_bang_plan_.progress_y += curr_vel_.y * update_dt_;
        }
    }
    return acc_cmd;
}

void Flyappy::updateMode()
{
    // estimate distance to asteroid
    if (!estimateDistanceToNextAsteriod())
    {
        // mode_ = kCruise;
        return;
    }
    if (distance_to_next_asteriod_ > 2.5)
    {
        mode_ = kCruise;
        return;
    }
    if (mode_ == kCruise)
    {
        mode_ = kSearching;
        search_timer_ = 0.0;
    }
    if (mode_ == kSearching)
    {
        search_timer_ += update_dt_;
        if (searchForConsistentFreeAngle())
        {
            updateBangBangPlan();
            search_started_ = true;
            found_target_ = true;
            mode_ = kBangBang;
        }
    }
    if (mode_ == kBangBang) {
        searchForConsistentFreeAngle();
    }

    // check condition for approaching
    if (fabs(steering_angle_) < 0.1)
    {
        mode_ = kApproaching;
        searchForConsistentFreeAngle();
        search_started_ = false;
    }

    if (mode_==kApproaching && fabs(steering_angle_) > 0.1) {
        mode_ = kSearching;
        search_timer_ = 0.0;
    }
}

void Flyappy::updateBangBangPlan()
{
    // according to the streering angle, update the bang bang plan
    // mode_ = kSearching;
    double threshold = 0.5;
    bang_bang_plan_.X = target_relative_pos_.x;
    bang_bang_plan_.Y = target_relative_pos_.y;
    bang_bang_plan_.progress_y = 0.0;
}

bool Flyappy::updateSteeringAngle()
{
    // Some rules: 1) If two consecutive laser scans are free, then pick the average angle
    // without hesitation.
    //             2) If there is no consecutive free laser scans, check the y gap between
    //             its neighbors and pick the one with the biggest gap. 3) If all the
    //             laser scans are blocked, then stick to the original plan and return
    //             false

    double max_gap = 0;
    double ans = 0.0;
    // look for directions that are not blocked by asteriods, change its neighbors and
    // calculate the Y gap
    for (int i = 0; i < curr_laser_scan_.size - 1; ++i)
    {
        if (checkIfFree(i) && checkIfFree(i + 1))
        {
            steering_angle_ =
                    (curr_laser_scan_.angles[i] + curr_laser_scan_.angles[i + 1]) / 2.0;
            target_relative_pos_.x = distance_to_next_asteriod_;
            target_relative_pos_.y =
                    (relative_distance_y_[i] + relative_distance_y_[i + 1]) / 2.0;
            std::cout << "Found a free direction Case 1 " << i << std::endl;
            return true;
        }
        // find a direction that is not blocked
        if (checkIfFree(i))
        {
            // do not consider the first and last laser scan for now
            if (i == 0) continue;
            // check if the Y gap is big enough
            double y_gap = relative_distance_y_[i + 1] - relative_distance_y_[i - 1];
            std::cout << "Y gap " << y_gap << std::endl;
            if (y_gap > safe_gap_)
            {
                // this direction is good
                steering_angle_ = curr_laser_scan_.angles[i];
                target_relative_pos_.x = distance_to_next_asteriod_;
                target_relative_pos_.y = relative_distance_y_[i];
                std::cout << "Found a free direction Case 2 " << i << std::endl;
                return true;
            }
        }
    }
    return false;
}

bool Flyappy::estimateDistanceToNextAsteriod()
{
    // estimate the distance to next column of asteriods by calculating the mean laser
    // range in x direction
    double sum = 0;
    int blocked_count = 0;

    if (curr_laser_scan_.size == 0) return false;

    relative_distance_x_.resize(curr_laser_scan_.size);
    relative_distance_x_.clear();
    relative_distance_y_.resize(curr_laser_scan_.size);
    relative_distance_y_.clear();
    for (int i = 0; i < curr_laser_scan_.size; ++i)
    {
        relative_distance_x_.push_back(curr_laser_scan_.ranges[i] *
                                       cos(curr_laser_scan_.angles[i]));
        relative_distance_y_.push_back(curr_laser_scan_.ranges[i] *
                                       sin(curr_laser_scan_.angles[i]));
        if (curr_laser_scan_.intensities[i] > 0 && curr_laser_scan_.ranges[i] < 2.5)
        {
            sum += curr_laser_scan_.ranges[i] * cos(curr_laser_scan_.angles[i]);
            blocked_count++;
        }
    }
    if (blocked_count < 4)
    {
        distance_to_next_asteriod_ = curr_laser_scan_.range_max;
    }
    else
    {
        distance_to_next_asteriod_ = sum / blocked_count;
    }
    // std::cout << "blocked_count: " << blocked_count << std::endl;
    return true;
}

bool Flyappy::checkIfFree(int index)
{
    // check if the angle[index] is not blocked by asteriods
    if (curr_laser_scan_.intensities[index] == 0) return true;
    // if the angle is blacked by wall/floor, and is still available
    if (curr_laser_scan_.ranges[index] * cos(curr_laser_scan_.angles[index]) >
        (distance_to_next_asteriod_ + threshold_))
        return true;
    return false;
}

void Flyappy::updateLaserScan(LaserScan& laser_scan)
{
    // update the laser scan, and push into the buffer of ranges.
    curr_laser_scan_ = laser_scan;
    range_buffer_.push_back(laser_scan.ranges);
    // a bit inefficient, but optimize later
    if (range_buffer_.size() == range_buffer_size_ + 1)
    {
        range_buffer_.erase(range_buffer_.begin());
    }
}

bool Flyappy::searchForConsistentFreeAngle()
{
    if (range_buffer_.size() < range_buffer_size_)
    {
        // wait until get enough samples
        return false;
    }
    // look for index that has all consecutive laser scans free
    for (int i = 0; i < curr_laser_scan_.size; ++i)
    {
        double sum = 0;
        for (int j = 0; j < range_buffer_size_; ++j)
        {
            sum += range_buffer_[j][i];
        }
        if (sum >= curr_laser_scan_.range_max * range_buffer_size_ - 1.0)
        {
            // found a free direction
            steering_angle_ = curr_laser_scan_.angles[i];
            target_relative_pos_.x = distance_to_next_asteriod_;
            target_relative_pos_.y = relative_distance_y_[i];
            // std::cout << "Found a free direction Case 3 " << i << std::endl;
            return true;
        }
    }
    return false;
}