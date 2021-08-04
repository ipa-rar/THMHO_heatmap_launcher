#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <pluginlib/class_loader.hpp>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "costmap_generator");

    std::string name = "costmap";
    std::string global_frame_ = "map";
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle g_nh;

    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    costmap_2d::LayeredCostmap* layered_costmap_ = new costmap_2d::LayeredCostmap(global_frame_, false, false);

    // create layers -- static and obstacle
    pluginlib::ClassLoader<costmap_2d::Layer> plugin_loader_("costmap_2d", "costmap_2d::Layer");
    boost::shared_ptr<costmap_2d::Layer> plugin_static = plugin_loader_.createInstance("costmap_2d::StaticLayer");
    layered_costmap_->addPlugin(plugin_static);
    plugin_static->initialize(layered_costmap_, name + "/" + "static_map", &buffer);
    plugin_static->activate();

    boost::shared_ptr<costmap_2d::Layer> plugin_obstacle = plugin_loader_.createInstance("costmap_2d::ObstacleLayer");
    layered_costmap_->addPlugin(plugin_obstacle);
    plugin_obstacle->initialize(layered_costmap_, name + "/" + "obstacles", &buffer);
    plugin_obstacle->activate();

    costmap_2d::Costmap2DPublisher *publisher_ = new costmap_2d::Costmap2DPublisher(&private_nh, layered_costmap_->getCostmap(), global_frame_, "costmap",
                                      true);

    double map_publish_frequency = 10;
    ros::Duration publish_cycle = ros::Duration(1 / map_publish_frequency);
    ros::Time last_publish = ros::Time::now();

    ros::Rate r(map_publish_frequency);
    while (g_nh.ok())
    {
        #ifdef HAVE_SYS_TIME_H
        struct timeval start, end;
        double start_t, end_t, t_diff;
        gettimeofday(&start, NULL);
        #endif

        layered_costmap_->updateMap(0, 0, 0);

        #ifdef HAVE_SYS_TIME_H
        gettimeofday(&end, NULL);
        start_t = start.tv_sec + double(start.tv_usec) / 1e6;
        end_t = end.tv_sec + double(end.tv_usec) / 1e6;
        t_diff = end_t - start_t;
        ROS_DEBUG("Map update time: %.9f", t_diff);
        #endif
        
        if (layered_costmap_->isInitialized())
        {
            unsigned int x0, y0, xn, yn;
            layered_costmap_->getBounds(&x0, &xn, &y0, &yn);
            publisher_->updateBounds(x0, xn, y0, yn);

            ros::Time now = ros::Time::now();
            if (last_publish + publish_cycle < now)
            {
                publisher_->publishCostmap();
                last_publish = now;
            }
        }
        r.sleep();
        // make sure to sleep for the remainder of our cycle time
        if (r.cycleTime() > ros::Duration(1 / map_publish_frequency))
        ROS_WARN("Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", map_publish_frequency,
                r.cycleTime().toSec());

        ros::spinOnce();
    }

    return 0;
}
