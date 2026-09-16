#include "gcopter/flatness.hpp"
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include "star/robust_star.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include <string>

class Sconvex
{
public:
    Sconvex(ros::NodeHandle &nh_, ros::NodeHandle &nh_private)
        : nh(nh_), mapInitialized(false)
    {

        std::string mapTopic, targetTopic;
        nh_private.getParam("MapTopic", mapTopic);
        nh_private.param("TargetTopic", targetTopic, std::string("/move_base_simple/goal"));
        nh_private.param("RunHz", runHz, 5.0);
        nh_private.getParam("InflateRange", inflateRange);

        stats.resize(1);
        Eigen::Vector3d map_size;
        mapBound.resize(6);
        nh_private.param("map/x_size", map_size(0), 40.0);
        nh_private.param("map/y_size", map_size(1), 40.0);
        nh_private.param("map/z_size", map_size(2), 5.0);
        nh_private.param("map/x_origin", mapBound[0], -20.0);
        nh_private.param("map/y_origin", mapBound[2], -20.0);
        nh_private.param("map/z_origin", mapBound[4], 0.0);
        mapBound[1] = mapBound[0] + map_size(0);
        mapBound[3] = mapBound[2] + map_size(1);
        mapBound[5] = mapBound[4] + map_size(2);

        std::vector<double> goal_param;
        nh_private.getParam("Goal", goal_param);
        if (goal_param.size() == 3)
        {
            seed_pt = Eigen::Vector3d(goal_param[0], goal_param[1], goal_param[2]);
        }

        star_map.initVisualizer(nh_private, 0);

        mapSub = nh.subscribe(mapTopic, 1, &Sconvex::handleMap, this,
                              ros::TransportHints().tcpNoDelay());
        targetSub = nh.subscribe(targetTopic, 1, &Sconvex::targetCallback, this,
                                 ros::TransportHints().tcpNoDelay());

        std::cout << "Sconvex (RobustStarManager) node initialized." << std::endl;


        double safe_hz = runHz;
        if (!std::isfinite(safe_hz) || safe_hz <= 0.0) {
            ROS_WARN("[single] Invalid RunHz %.6f, using 1.0", safe_hz);
            safe_hz = 1.0;
        }
        double period = 1.0 / safe_hz;
        if (!std::isfinite(period) || period <= 0.0) {
            ROS_WARN("[single] Invalid timer period %.6f, using 1.0", period);
            period = 1.0;
        }
        if (period > 10.0) period = 10.0;
        runTimer = nh.createTimer(ros::Duration(period),
                                  &Sconvex::timerCallback, this);

        std::cout << "Interactive mode: click \"2D Nav Goal\" in rviz (topic \""
                  << targetTopic << "\") to set the query point." << std::endl;

        const Eigen::Vector3d lowCorner(mapBound[0], mapBound[2], mapBound[4]);
        const Eigen::Vector3d highCorner(mapBound[1], mapBound[3], mapBound[5]);

        star_map.init(8.0f * inflateRange, 0.05, lowCorner, highCorner,
                      inflateRange, safe_hz);

    }

private:
    struct StatSeries
    {
        std::vector<double> times_ms;
        std::vector<double> volumes;
        std::vector<int> faces;
    };

    ros::NodeHandle nh;
    ros::Subscriber mapSub;
    ros::Subscriber targetSub;
    ros::Timer runTimer;

    bool mapInitialized{false};
    bool haveGoal{false};
    std::vector<Eigen::Vector3d> mapPoints;
    Eigen::Vector3d seed_pt;
    std::vector<double> mapBound;
    double inflateRange{0.0};
    double runHz{5.0};
    std::vector<StatSeries> stats;
    Star::RobustStarManager star_map;

    void recordStats(size_t idx, double time_ms, double volume, int faces_num)
    {
        if (idx >= stats.size()) { return; }
        stats[idx].times_ms.push_back(time_ms);
        stats[idx].volumes.push_back(volume);
        stats[idx].faces.push_back(faces_num);
    }

    void handleMap(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        mapPoints.clear();
        if (msg->point_step == 0 || msg->data.empty())
        {
            return;
        }
        const size_t total = msg->data.size() / msg->point_step;
        const float *fdata = reinterpret_cast<const float *>(&msg->data[0]);
        mapPoints.reserve(total);
        for (size_t i = 0; i < total; i++)
        {
            const size_t cur = msg->point_step / sizeof(float) * i;
            if (!std::isfinite(fdata[cur + 0]) || !std::isfinite(fdata[cur + 1]) || !std::isfinite(fdata[cur + 2]))
            {
                continue;
            }
            const Eigen::Vector3d pt(fdata[cur + 0], fdata[cur + 1], fdata[cur + 2]);
            mapPoints.push_back(pt);
        }

        mapInitialized = true;
    }

    void targetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // set z as the mid-height of the map, since "2D Nav Goal" only gives (x, y)
        seed_pt = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y,
                                  (mapBound[4] + mapBound[5]) * 0.5);
        haveGoal = true;

        std::cout << "[star] New seed point: " << seed_pt.transpose() << std::endl;
        run();
    }

    void timerCallback(const ros::TimerEvent &)
    {
        if (!mapInitialized || !haveGoal) { return; }
        run();
    }

    void run()
    {
        if (!mapInitialized || !haveGoal) { return;}

        std::cout << "\033[33m"
                  << "[star] Running SFC optimization at seed point: "
                  << seed_pt.transpose()
                  << "\033[0m" << std::endl;

        //get local bounded point cloud
        const auto t1 = std::chrono::high_resolution_clock::now();

        const int update_status = star_map.update(seed_pt, mapPoints);

        const auto t2 = std::chrono::high_resolution_clock::now();
        const double time_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
        //print in blue
        std::cout << "\033[34m"
                  << "[star] one polytope took "
                  << time_ms
                  << " ms, update status: " << update_status
                  << "\033[0m" << std::endl;

        //visualization
        star_map.visualizeCurrent(seed_pt);
        //run evaluation, just print the volume of the polytope
        if (const auto *current = star_map.getCurrentStarNode())
        {
            if (current->star_kernel.rows() > 0)
            {
                Eigen::Matrix3Xd kernel_mesh;
                sfc_utils::getMesh(current->star_kernel, kernel_mesh);
                double volume = 0.0;
                sfc_utils::getMeshVolume(kernel_mesh, volume);
                const int faces_num = static_cast<int>(current->star_kernel.rows());
                recordStats(0, time_ms, volume, faces_num);
                std::cout << "[star] polytope volume: " << volume  << std::endl;
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sconvex_node");
    ros::NodeHandle nh, nh_private("~");
    Sconvex planner(nh, nh_private);
    ros::spin();
    return 0;
}
