#ifndef SFC_VIS_HPP
#define SFC_VIS_HPP

#include <iostream>
#include <memory>
#include <chrono>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "gcopter/trajectory.hpp"
#include "gcopter/quickhull.hpp"

#include "sfc/sfc_utils.hpp"


class sfc_vis
{
private:
    // config contains the scale for some markers
    ros::NodeHandle nh;
    // These are publishers for path, waypoints on the trajectory,
    // the entire trajectory, the mesh of free-space polytopes,
    // the edge of free-space polytopes, and spheres for safety radius
    ros::Publisher routePub;
    ros::Publisher wayPointsPub;
    ros::Publisher trajectoryPub;
    ros::Publisher meshPub;
    ros::Publisher edgePub;
    ros::Publisher spherePub;
    ros::Publisher textPub;
    ros::Publisher ellipPub;

    std::string frame_id;

    int id_;
    Eigen::Vector3d color;

public:

    sfc_vis(){}
    
    sfc_vis(ros::NodeHandle &nh_, int id = 0)
        : nh(nh_), id_(id)
    {
        std::string visualizer_topic = "/visualizer" + std::to_string(id_);
        //color will be related to the id. use id/10 to get the color
        color(0) = std::max((id% 2) / 2.0, 0.2);
        color(1) = std::min((id % 5) / 5.0, 0.9);
        color(2) = (id % 3) / 3.0;

        routePub = nh.advertise<visualization_msgs::Marker>(visualizer_topic + "/route", 10);
        wayPointsPub = nh.advertise<visualization_msgs::Marker>(visualizer_topic + "/waypoints", 10);
        trajectoryPub = nh.advertise<visualization_msgs::Marker>(visualizer_topic + "/trajectory", 10);
        meshPub = nh.advertise<visualization_msgs::Marker>(visualizer_topic + "/mesh", 1000);
        edgePub = nh.advertise<visualization_msgs::Marker>(visualizer_topic + "/edge", 1000);
        ellipPub = nh.advertise<visualization_msgs::MarkerArray>(visualizer_topic + "/ellipsoids", 1000);
        spherePub = nh.advertise<visualization_msgs::Marker>(visualizer_topic + "/spheres", 1000);
        textPub = nh.advertise<visualization_msgs::Marker>(visualizer_topic + "/text", 1000);

        nh.param("frame_id", frame_id, std::string("map"));
        std::cout << "frame_id  is " <<frame_id<< std::endl;

    }

    inline void clearAll()
    {
        visualization_msgs::Marker empty_marker;
        visualization_msgs::MarkerArray empty_array;
        
        empty_marker.header.stamp = ros::Time::now();
        empty_marker.header.frame_id = frame_id;
        empty_marker.id = 0;
        empty_marker.action = visualization_msgs::Marker::DELETEALL;
        empty_array.markers.push_back(empty_marker);
        ellipPub.publish(empty_array);

        ros::Duration(0.5).sleep();

    }


    // for collision free route
    inline void visualizePath(std::vector<Eigen::Vector3d> &route)
    {
        Eigen::Vector3d wp_color;
        wp_color << color(2), color(1), color(0);
        visLinePath(route, routePub, wp_color);
        
    }
    
    inline void visualizeWaypoints(std::vector<Eigen::Vector3d> &waypoints)
    {
        visLinePath(waypoints, wayPointsPub, color);
    }

   
    inline void visLinePath(const std::vector<Eigen::Vector3d> &route,
                        ros::Publisher &routePub,
                        Eigen::Vector3d pathcolor = Eigen::Vector3d(0.0, 0.0, 1.0))
    {
        visualization_msgs::Marker routeMarker, sphereMarker;

        routeMarker.id = 0;
        routeMarker.type = visualization_msgs::Marker::LINE_LIST;
        routeMarker.header.stamp = ros::Time::now();
        routeMarker.header.frame_id = frame_id;
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = pathcolor(0);
        routeMarker.color.g = pathcolor(1);
        routeMarker.color.b = pathcolor(2);
        routeMarker.color.a = 1.0;
        routeMarker.scale.x = 0.07;
        routeMarker.scale.y = 0.07;
        routeMarker.scale.z = 0.02;

        sphereMarker = routeMarker;
        sphereMarker.id = 1000;
        sphereMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        sphereMarker.action = visualization_msgs::Marker::ADD;
        sphereMarker.ns = "route_point";
        sphereMarker.scale.x = 0.15;
        sphereMarker.scale.y = 0.15;
        sphereMarker.scale.z = 0.15;


        if (route.size() > 0)
        {   
            Eigen::Vector3d lastX = route[0];
            for (size_t i = 1; i < route.size(); i++)
            {
                geometry_msgs::Point point;
                point.x = lastX(0);
                point.y = lastX(1);
                point.z = lastX(2);
                routeMarker.points.push_back(point);
                point.x = route[i](0);
                point.y = route[i](1);
                point.z = route[i](2);
                routeMarker.points.push_back(point);
                lastX = route[i];
                sphereMarker.points.push_back(point);
            }

            routePub.publish(routeMarker);
            routePub.publish(sphereMarker);
        }
    }


    // Visualize the trajectory and its front-end path
    template <int D, class Temp>
    inline void visualize(const Trajectory<D> &traj,
                          const Temp &route)
    {
        visualization_msgs::Marker wayPointsMarker, trajMarker;

        visLinePath(route, wayPointsPub, Eigen::Vector3d(0.5, 0.5, 0.0));

        wayPointsMarker.header.stamp = ros::Time::now();
        wayPointsMarker.header.frame_id = frame_id;
        wayPointsMarker.pose.orientation.w = 1.00;
        wayPointsMarker.action = visualization_msgs::Marker::ADD;
        wayPointsMarker.id = -wayPointsMarker.id - 1;
        wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 0.70;
        wayPointsMarker.color.g = 0.10;
        wayPointsMarker.color.b = 0.40;
        wayPointsMarker.color.a = 0.8;
        wayPointsMarker.scale.x = 0.3;
        wayPointsMarker.scale.y = 0.3;
        wayPointsMarker.scale.z = 0.3;

        trajMarker = wayPointsMarker;
        trajMarker.type = visualization_msgs::Marker::LINE_LIST;
        trajMarker.id = 0;
        trajMarker.ns = "trajectory";
        trajMarker.color.r = 0.16;
        trajMarker.color.g = 0.50;
        trajMarker.color.b = 0.36;
        trajMarker.scale.x = 0.15;
        trajMarker.scale.y = 0.15;
        trajMarker.scale.z = 0.15;

        if (traj.getPieceNum() > 0)
        {
            Eigen::MatrixXd wps = traj.getPositions();
            for (int i = 0; i < wps.cols(); i++)
            {
                geometry_msgs::Point point;
                point.x = wps.col(i)(0);
                point.y = wps.col(i)(1);
                point.z = wps.col(i)(2);
                wayPointsMarker.points.push_back(point);
            }

            wayPointsPub.publish(wayPointsMarker);
        }

        if (traj.getPieceNum() > 0)
        {
            double T = 0.1;
            Eigen::Vector3d lastX = traj.getPos(0.0);
            for (double t = T; t < traj.getTotalDuration(); t += T)
            {
                geometry_msgs::Point point;
                Eigen::Vector3d X = traj.getPos(t);
                point.x = lastX(0);
                point.y = lastX(1);
                point.z = lastX(2);
                trajMarker.points.push_back(point);
                point.x = X(0);
                point.y = X(1);
                point.z = X(2);
                trajMarker.points.push_back(point);
                lastX = X;
            }
            
        }
        trajectoryPub.publish(trajMarker);
    }


    // Visualize some polytopes in H-representation
    inline void visualizePolytope(const std::vector<Eigen::MatrixX4d> &hPolys)
    {

        // Due to the fact that H-representation cannot be directly visualized
        // We first conduct vertex enumeration of them, then apply quickhull
        // to obtain triangle meshs of polyhedra
        Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);
        for (size_t id = 0; id < hPolys.size(); id++)
        {
            oldTris = mesh;
            Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
            sfc_utils::getMesh(hPolys[id], curTris);
            mesh.resize(3, oldTris.cols() + curTris.cols());
            mesh.leftCols(oldTris.cols()) = oldTris;
            mesh.rightCols(curTris.cols()) = curTris;
        }

        // RVIZ support tris for visualization
        visualization_msgs::Marker meshMarker, edgeMarker;

        meshMarker.id = 0;
        meshMarker.header.stamp = ros::Time::now();
        meshMarker.header.frame_id = frame_id;
        meshMarker.pose.orientation.w = 1.00;
        meshMarker.action = visualization_msgs::Marker::ADD;
        meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        meshMarker.ns = "mesh";
        meshMarker.color.r = 0.6;
        meshMarker.color.g = 0.15;
        meshMarker.color.b = 0.0;
        meshMarker.color.a = 0.15;
        meshMarker.scale.x = 1.0;
        meshMarker.scale.y = 1.0;
        meshMarker.scale.z = 1.0;

        edgeMarker = meshMarker;
        edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
        edgeMarker.ns = "edge";
        edgeMarker.color.r = 0.6;
        edgeMarker.color.g = 0.12;
        edgeMarker.color.b = 0.0;
        edgeMarker.color.a = 0.05;
        edgeMarker.scale.x = 0.02;

        geometry_msgs::Point point;

        int ptnum = mesh.cols();

        for (int i = 0; i < ptnum; i++)
        {
            point.x = mesh(0, i);
            point.y = mesh(1, i);
            point.z = mesh(2, i);
            meshMarker.points.push_back(point);
        }

        for (int i = 0; i < ptnum / 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                point.x = mesh(0, 3 * i + j);
                point.y = mesh(1, 3 * i + j);
                point.z = mesh(2, 3 * i + j);
                edgeMarker.points.push_back(point);
                point.x = mesh(0, 3 * i + (j + 1) % 3);
                point.y = mesh(1, 3 * i + (j + 1) % 3);
                point.z = mesh(2, 3 * i + (j + 1) % 3);
                edgeMarker.points.push_back(point);
            }
        }

        meshPub.publish(meshMarker);
        edgePub.publish(edgeMarker);

        return;
    }


    void visualizePolytope2D(const std::vector<Eigen::Matrix2Xd> &allTris)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;   // Change to your frame
        marker.header.stamp = ros::Time::now();
        marker.ns = "polytope";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05;  // line width
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.r = 1.0f;
        marker.color.g = 0.6f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marker.pose.orientation.w = 1.0; // No rotation


        // // Clear points
        // marker.points.clear();

        // For each polygon in allTris
        for (size_t i = 0; i < allTris.size(); ++i)
        {
            const auto& poly = allTris[i];
            size_t n = poly.cols();

            std::cout << "Polygon " << i << " has " << n << " points." << std::endl;

            // Add edges between consecutive points and close polygon
            for (size_t j = 0; j < n; ++j)
            {
                geometry_msgs::Point p1, p2;
                p1.x = poly(0, j);
                p1.y = poly(1, j);
                p1.z = 0.1;

                size_t next_idx = (j + 1) % n;
                p2.x = poly(0, next_idx);
                p2.y = poly(1, next_idx);
                p2.z = 0.1;

                marker.points.push_back(p1);
                marker.points.push_back(p2);
            }
        }


        edgePub.publish(marker);
    }


    inline void visualizeEllipsoids(const std::vector<Eigen::Matrix3d> &Rs,
                                   const std::vector<Eigen::Vector3d> &ds,
                                   const std::vector<Eigen::Vector3d> &rs)
    {
        visualization_msgs::Marker ellipMarker;
        visualization_msgs::MarkerArray ellipMarkers;

        //std::cout << "Rs size: " << Rs.size() << std::endl;


        ellipMarker.header.stamp = ros::Time::now();
        ellipMarker.header.frame_id = frame_id;
        ellipMarker.id = 0;
        ellipMarker.action = visualization_msgs::Marker::DELETEALL;
        ellipPub.publish(ellipMarkers);

        ellipMarker.header.stamp = ros::Time::now();
        ellipMarker.action = visualization_msgs::Marker::ADD;
        ellipMarker.type = visualization_msgs::Marker::SPHERE;
        ellipMarker.id = 1;
        ellipMarker.pose.orientation.w = 1.00;
        ellipMarker.ns = "ellipsoids";

        //blue
        ellipMarker.color.r = 0.00;
        ellipMarker.color.g = 0.30;
        ellipMarker.color.b = 0.60;
        ellipMarker.color.a = 0.25;
        
        for (size_t i = 0; i < Rs.size(); i++)
        {
            Eigen::Matrix3d R = Rs[i];
            Eigen::Vector3d d = ds[i];
            Eigen::Vector3d r = rs[i];
            
            ellipMarker.scale.x = r(0) * 2.0;
            ellipMarker.scale.y = r(1) * 2.0;
            ellipMarker.scale.z = r(2) * 2.0;

            ellipMarker.pose.position.x = d(0);
            ellipMarker.pose.position.y = d(1);
            ellipMarker.pose.position.z = d(2);

            //orientation is from R
            Eigen::Quaterniond q(R);
            ellipMarker.pose.orientation.x = q.x();
            ellipMarker.pose.orientation.y = q.y();
            ellipMarker.pose.orientation.z = q.z();
            ellipMarker.pose.orientation.w = q.w();

            ellipMarker.id += 1;

            ellipMarkers.markers.push_back(ellipMarker);
        }

        //std::cout << "ellipMarkers size: " << ellipMarkers.markers.size() << std::endl;

        ellipPub.publish(ellipMarkers);
    }






    inline void visualizeText(const std::string &str, 
                              const Eigen::Vector3d &pos = Eigen::Vector3d(-15.0, -7.0, 0.0))
    {
        visualization_msgs::Marker textMaker;
        textMaker.id = 0;
        textMaker.header.stamp = ros::Time::now();
        textMaker.header.frame_id = frame_id;
        textMaker.pose.orientation.w = 1.00;
        textMaker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textMaker.action = visualization_msgs::Marker::MODIFY;
        textMaker.pose.position.x = pos(0);
        textMaker.pose.position.y = pos(1);
        textMaker.pose.position.z = pos(2);
        textMaker.text = str;
        textMaker.color.r = 0.00;
        textMaker.color.g = 0.00;
        textMaker.color.b = 0.00;
        textMaker.color.a = 1.00;
        textMaker.scale.x = 0.90;
        textMaker.scale.y = 0.90;
        textMaker.scale.z = 0.90;


        textPub.publish(textMaker);

        return;
    }



    // Visualize all spheres with centers sphs and the same radius
    inline void visualizeSphere(const Eigen::Vector3d &center,
                                const double &radius)
    {
        visualization_msgs::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = 0;
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = frame_id;
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::ADD;
        sphereMarkers.ns = "spheres";
        sphereMarkers.color.r = 0.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 1.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::Marker::DELETE;

        geometry_msgs::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);

        spherePub.publish(sphereDeleter);
        spherePub.publish(sphereMarkers);
    }

    inline void visualizeStartGoal(const Eigen::Vector3d &center,
                                   const double &radius,
                                   const int sg,
                                   Eigen::Vector3d color = Eigen::Vector3d(0.0, 0.3, 0.5))
    {
        visualization_msgs::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = sg;
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = frame_id;
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::ADD;
        sphereMarkers.ns = "StartGoal";
        sphereMarkers.color.r = color(0);
        sphereMarkers.color.g = color(1);
        sphereMarkers.color.b = color(2);
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::Marker::DELETEALL;

        geometry_msgs::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);

        if (sg == 0)
        {
            spherePub.publish(sphereDeleter);
            ros::Duration(1.0e-9).sleep();
            sphereMarkers.header.stamp = ros::Time::now();
        }
        spherePub.publish(sphereMarkers);
    }
};

#endif