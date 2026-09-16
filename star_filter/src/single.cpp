#include "gcopter/flatness.hpp"
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include "star/star.hpp"
#include "sfc/sfc_vis.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include <string>

class Single
{
public:
    Single(ros::NodeHandle &nh_, ros::NodeHandle &nh_private)
        : nh(nh_)
    {
        std::string mapTopic, targetTopic;
        nh_private.getParam("MapTopic", mapTopic);
        nh_private.param("TargetTopic", targetTopic, std::string("/move_base_simple/goal"));
        nh_private.param("InflateRange", inflateRange, 3.0);
        nh_private.param("PathOffsetZ", pathOffsetZ, 1.0);
        nh_private.param("MaxIter", maxIter, 5);

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

        visualizer.reset(new sfc_vis(nh_private, 0));

        mapSub = nh.subscribe(mapTopic, 1, &Single::handleMap, this,
                              ros::TransportHints().tcpNoDelay());
        targetSub = nh.subscribe(targetTopic, 1, &Single::targetCallback, this,
                                 ros::TransportHints().tcpNoDelay());

        std::cout << "Single node initialized. Click \"2D Nav Goal\" in rviz (topic \""
                  << targetTopic << "\") to generate one polytope." << std::endl;
    }

private:
    // FIRI's initial (FiriInitMode::Thin) seed radius, as a fraction of
    // InflateRange -- see targetCallback for why this matters for speed/quality.
    static constexpr double kSeedRadiusFraction = 0.3;

    ros::NodeHandle nh;
    ros::Subscriber mapSub;
    ros::Subscriber targetSub;
    std::unique_ptr<sfc_vis> visualizer;

    bool mapInitialized{false};
    std::vector<Eigen::Vector3d> mapPoints;
    std::vector<double> mapBound;
    double inflateRange{3.0};
    double pathOffsetZ{1.0};
    int maxIter{5};

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

    // Axis-aligned box around query_point, clipped to the map bounds, plus the
    // map points that fall inside it.
    bool buildBoundedPointCloud(const Eigen::Vector3d &query_point,
                                Eigen::Matrix<double, 6, 4> &bd,
                                Eigen::Matrix3Xd &pc) const
    {
        const Eigen::Vector3d lowCorner(mapBound[0], mapBound[2], mapBound[4]);
        const Eigen::Vector3d highCorner(mapBound[1], mapBound[3], mapBound[5]);
        const Eigen::Vector3d lowerBound =
            (query_point.array() - inflateRange).max(lowCorner.array());
        const Eigen::Vector3d upperBound =
            (query_point.array() + inflateRange).min(highCorner.array());

        bd.setZero();
        bd.col(0) << 1.0, -1.0, 0.0, 0.0, 0.0, 0.0;
        bd.col(1) << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0;
        bd.col(2) << 0.0, 0.0, 0.0, 0.0, 1.0, -1.0;
        bd.col(3) << -upperBound(0), lowerBound(0),
                     -upperBound(1), lowerBound(1),
                     -upperBound(2), lowerBound(2);

        std::vector<Eigen::Vector3d> valid_pc;
        valid_pc.reserve(mapPoints.size());
        for (const Eigen::Vector3d &p : mapPoints)
        {
            if ((p.array() >= lowerBound.array()).all() && (p.array() <= upperBound.array()).all())
            {
                valid_pc.push_back(p);
            }
        }
        if (valid_pc.empty())
        {
            return false;
        }
        pc.resize(3, static_cast<Eigen::Index>(valid_pc.size()));
        for (size_t i = 0; i < valid_pc.size(); ++i)
        {
            pc.col(static_cast<Eigen::Index>(i)) = valid_pc[i];
        }
        return true;
    }

    void targetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (!mapInitialized)
        {
            std::cout << "[single] Map not ready yet, ignoring click." << std::endl;
            return;
        }

        const double mid_z = 0.5 * (mapBound[4] + mapBound[5]);
        const Eigen::Vector3d a(msg->pose.position.x, msg->pose.position.y, mid_z);
        const Eigen::Vector3d b = a + Eigen::Vector3d(0.0, 0.0, pathOffsetZ);

        Eigen::Matrix<double, 6, 4> bd;
        Eigen::Matrix3Xd local_pc;
        if (!buildBoundedPointCloud(a, bd, local_pc))
        {
            std::cout << "[single] No points found near " << a.transpose()
                      << "; skipping." << std::endl;
            return;
        }

        Eigen::MatrixX4d hpoly;
        Eigen::Matrix3d R;
        Eigen::Vector3d d, r;

        const auto t1 = std::chrono::high_resolution_clock::now();

        corridor.firiFiltered(a, b, local_pc, bd, hpoly, R, d, r, maxIter,
                              1.0e-6, 0.02, Star::Corridor::FiriInitMode::Thin,
                              kSeedRadiusFraction * inflateRange);
        const auto t2 = std::chrono::high_resolution_clock::now();
        const double time_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();

        std::cout << "\033[34m"
                  << "[single] one polytope took " << time_ms
                  << " ms, " << hpoly.rows() << " faces, "
                  << corridor.lastTiming().loops << " FIRI loops"
                  << "\033[0m" << std::endl;

        visualizer->clearAll();
        visualizer->visualizeStartGoal(a, 0.3, 0);
        visualizer->visualizeStartGoal(b, 0.3, 1);
        visualizer->visualizePolytope({hpoly});

        Eigen::Matrix3Xd mesh;
        sfc_utils::getMesh(hpoly, mesh);
        double volume = 0.0;
        sfc_utils::getMeshVolume(mesh, volume);
        std::cout << "[single] polytope volume: " << volume << std::endl;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_node");
    ros::NodeHandle nh, nh_private("~");
    Single node(nh, nh_private);
    ros::spin();
    return 0;
}
