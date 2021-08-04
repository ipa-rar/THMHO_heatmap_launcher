#include <iostream>
#include <mutex>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"


class CostmapAggregator{

public:
    CostmapAggregator() : seq_(0), num_scans_(0)
    {
        heatmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/heatmap", 1000);
        timer_ = nh_.createTimer(ros::Duration(60.0), &CostmapAggregator::publishHeatmap, this);
        costmap_sub_ = nh_.subscribe("/costmap_generator/costmap/costmap", 10, &CostmapAggregator::costmapCallback, this);
        occupancy_grid_ = boost::make_shared<nav_msgs::OccupancyGrid>();
    }

private:
    uint64_t seq_;
    uint64_t num_scans_;
    ros::NodeHandle nh_;
    ros::Publisher heatmap_pub_;
    ros::Subscriber costmap_sub_;
    ros::Timer timer_;
    std::mutex heatmap_mutex_;
    boost::shared_ptr<nav_msgs::OccupancyGrid> occupancy_grid_;

    const float max_costmap_ = 100.0f;   // scaling factor for obstacle "hits"
    std::vector<float> heatmap_vec_;

    void publishHeatmap(const ros::TimerEvent& e) {
        std::uint32_t width = occupancy_grid_->info.width;
        std::uint32_t height = occupancy_grid_->info.height;
        occupancy_grid_->data.resize(width * height);

        {   // scope for heatmap_mutex_
            std::lock_guard<std::mutex> lock(heatmap_mutex_);
            for(int i = 0; i < width; i++)
            {
                for(int j = 0; j < height; j++)
                {
                    int idx = i + j * width;
                    // normalize the heatmap values to max_costmap
                    occupancy_grid_->data[idx] = static_cast<int8_t>(heatmap_vec_[idx] / (float)num_scans_ * max_costmap_);
                }
            }
            heatmap_vec_.clear();
            num_scans_ = 0;
        }   // end of scope for heatmap_mutex_

        occupancy_grid_->header.stamp = ros::Time::now();
        occupancy_grid_->header.seq = seq_;
        heatmap_pub_.publish(occupancy_grid_);
        
        occupancy_grid_ = boost::make_shared<nav_msgs::OccupancyGrid>();
        seq_++;
    }

    void costmapCallback(const nav_msgs::OccupancyGrid::Ptr& msg)
    {
        if(heatmap_vec_.size() == 0)
        {
            occupancy_grid_->header.frame_id = msg->header.frame_id;
            occupancy_grid_->info = msg->info;
            heatmap_vec_.resize(msg->data.size(), 0);
        }

        {   // scope for heatmap_mutex_
            std::lock_guard<std::mutex> lock(heatmap_mutex_);
            for(int i = 0; i < msg->info.width; i++)
            {
                for(int j = 0; j < msg->info.height; j++)
                {
                    int idx = i + j * msg->info.width;
                    heatmap_vec_[idx] = heatmap_vec_[idx] + (float)msg->data[idx] / max_costmap_;
                }
            }
            num_scans_++;
        }   // end of scope for heatmap_mutex_
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "costmap_aggregator");
    CostmapAggregator costmap_aggregator;
    ros::spin();
}