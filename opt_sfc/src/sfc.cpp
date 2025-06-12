#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"

#include <ros/console.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

#include "sfc/sfc_cover_opt.hpp"
#include "sfc/plan_path.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <dirent.h>

class SFCServer
{
private:
    
    //ros related
    ros::NodeHandle nh;
    ros::Subscriber mapSub, targetSub, eciSub;
    ros::Timer timer;

    std::string mapTopic, targetTopic;


    bool mapInitialized;
    voxel_map::VoxelMap voxelMap;
    sfc_vis visualizer;
    std::vector<Eigen::Vector3d> startGoal;

    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    int optOrder;
    double inflateRange;
    int mode;
    int testNum, maxIter;
    int curTestNum = 0;
    std::vector<double> distRange;

    //get random position in the map
    std::uniform_real_distribution<double> rand_x_;
    std::uniform_real_distribution<double> rand_y_;
    std::uniform_real_distribution<double> rand_z_;
    std::default_random_engine eng_;

    sco::OptConvexCover opt_convex_cover;
    int map_id = 0;


public:
    SFCServer(ros::NodeHandle &nh_, ros::NodeHandle &nh_private)
        : nh(nh_), mapInitialized(false), opt_convex_cover(nh_private)
    {
        nh_private.getParam("MapTopic", mapTopic);
        nh_private.getParam("TargetTopic", targetTopic);
        nh_private.getParam("OptOrder", optOrder);
        nh_private.getParam("InflateRange", inflateRange);
        nh_private.getParam("Mode", mode);
        nh_private.getParam("TestNum", testNum);

        distRange.resize(2);
        nh_private.getParam("DistRange", distRange);
        nh_private.getParam("MaxIter", maxIter);

        //seed number
        int seed;
        nh_private.param("Seed", seed, 0);
        visualizer = sfc_vis(nh_private, 0);
        opt_convex_cover.setVisualizer(visualizer);


        Eigen::Vector3d map_size;
        mapBound.resize(6);

        nh_private.param("map/x_size", map_size(0), 40.0);
        nh_private.param("map/y_size", map_size(1), 40.0);
        nh_private.param("map/z_size", map_size(2), 5.0);
        nh_private.param("map/x_origin", mapBound[0], -20.0);
        nh_private.param("map/y_origin", mapBound[2], -20.0);
        nh_private.param("map/z_origin", mapBound[4], 0.0);
        nh_private.param("map/resolution", voxelWidth, 0.1);
        nh_private.param("map/inflate_radius", dilateRadius, 0.1);

        mapBound[1] = mapBound[0] + map_size(0);
        mapBound[3] = mapBound[2] + map_size(1);
        mapBound[5] = mapBound[4] + map_size(2);

        mapSub = nh.subscribe(mapTopic, 1, &SFCServer::mapCallBack, this,
                              ros::TransportHints().tcpNoDelay());

        targetSub = nh.subscribe(targetTopic, 1, &SFCServer::targetCallBack, this,
                                    ros::TransportHints().tcpNoDelay()); 


        std::cout << "seed: " << seed << std::endl;
        eng_ = std::default_random_engine(seed);
        rand_x_ = std::uniform_real_distribution<double>(mapBound[0], mapBound[1]);
        rand_y_ = std::uniform_real_distribution<double>(mapBound[2], mapBound[3]);
        rand_z_ = std::uniform_real_distribution<double>(mapBound[4], mapBound[5]);


        std::cout << "Planner Server Initialized !!!\n";
    }


    inline void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        const Eigen::Vector3i xyz((mapBound[1] - mapBound[0]) / voxelWidth,
                                (mapBound[3] - mapBound[2]) / voxelWidth,
                                (mapBound[5] - mapBound[4]) / voxelWidth);

        const Eigen::Vector3d offset(mapBound[0], mapBound[2], mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, voxelWidth);

        size_t cur = 0;
        const size_t total = msg->data.size() / msg->point_step;
        float *fdata = (float *)(&msg->data[0]);
        for (size_t i = 0; i < total; i++)
        {
            cur = msg->point_step / sizeof(float) * i;

            if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
            {
                continue;
            }
            voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                    fdata[cur + 1],
                                                    fdata[cur + 2]));
        }

        voxelMap.dilate(std::ceil(dilateRadius / voxelMap.getScale()));

        mapInitialized = true;
        map_id += 1;


        //call oneSampleTest
        if (mode == 1)
        {
            for (int i = 0; i < 20; i++)
            {   
                oneSampleTest();
            }
        }else
        {
            for (int i = 0; i < testNum; i++)
            {   
                oneSampleTest();
            }
        }
    }

    inline void plan(voxel_map::VoxelMap &curMap)
    {   

        if (mode >= 1)
        {
            if (curTestNum == testNum)
            {
                ROS_INFO("All Tests Finished !!!\n");
                curTestNum += 1;
                return;
            }
        }

        visualizer.clearAll();
        visualizer.visualizeStartGoal(startGoal[0], 0.3, 1, Eigen::Vector3d(0.9, 0.8, 0.2));
        visualizer.visualizeStartGoal(startGoal[1], 0.3, 2);

        if (startGoal.size() == 2)
        {
            Eigen::MatrixXd iniState(3, 3), finState(3, 3);

            iniState << startGoal.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
            finState << startGoal.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
            printf("\033[32m ======================== New Try : %d ======================== \033[0m\n", curTestNum);
            std::cout << "Start: " << startGoal.front().transpose() << std::endl;
            std::cout << "Goal: " << startGoal.back().transpose() << std::endl;

            /***step one: generate a init path***/
            std::vector<Eigen::Vector3d> route;
            plan_path::planPath<voxel_map::VoxelMap>(startGoal[0],
                                                    startGoal[1],
                                                    curMap.getOrigin(),
                                                    curMap.getCorner(),
                                                    &curMap, 0.1, 0.2,
                                                    route);
            
            if (route.size() <= 2)
            {
                ROS_WARN("No Path Found or Not enough points!!!\n");
                return;
            }
            visualizer.visualizePath(route);

            /***step two: corridor generation***/
            std::vector<Eigen::Vector3d> pc;
            curMap.getSurf(pc);

            std::vector<Eigen::MatrixX4d> hpolys;
            std::vector<Eigen::Matrix3d> Rs;
            std::vector<Eigen::Vector3d> ds;
            std::vector<Eigen::Vector3d> rs;
            auto t1 = std::chrono::high_resolution_clock::now();
            //ros::Time t = ros::Time::now();
            
            printf("\033[32m ================ opt_convex_cover: \033[0m");
            opt_convex_cover.convexCover(iniState, finState, pc,
                                        curMap.getOrigin(), 
                                        curMap.getCorner(),             
                                        inflateRange,
                                        route, 
                                        hpolys);

            auto t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> fp_ms = t2 - t1;
            std::cout << "convexCover time: " << fp_ms.count() << std::endl;
            //std::cout << "ros time: " << (ros::Time::now() - t).toSec() << std::endl;
            visualizer.visualizePolytope(hpolys);

            if (hpolys.size() == 0)
            {
                ROS_WARN("No Corridor Found !!!\n");
                return;
            }

            Eigen::VectorXd evals(4);
            std::vector<Eigen::Vector3d> inner_pts;
            bool valid = sfc_utils::evaluatehPolys(hpolys, startGoal[0], startGoal[1], evals, inner_pts);

            std::cout << "Eval: " << evals.transpose() << std::endl;

            if (valid == false)
            {
                std::cout << "No valid corridor found !!!\n";
            }

            curTestNum += 1;

        }

        return;
    }



    inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (mapInitialized)
        {
            if (startGoal.size() >= 2)
            {
                std::cout << "Clear Start Goal !!!\n";
                startGoal.clear();
            }
            const double zGoal = mapBound[4] + dilateRadius +
                                 fabs(msg->pose.orientation.z) *
                                     (mapBound[5] - mapBound[4] - 2 * dilateRadius);
            const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);
            if (voxelMap.query(goal) == 0)
            {
                visualizer.visualizeStartGoal(goal, 0.3, startGoal.size());
                startGoal.emplace_back(goal);
                std::cout << "Goal: " << goal.transpose() << std::endl;
            }
            else
            {
                ROS_WARN("Infeasible Position Selected !!!\n");
            }

            if (startGoal.size() == 2)
            {
                plan(voxelMap);
            }

            
        }
        return;
    }

    inline void oneSampleTest()
    {    
        if (curTestNum > testNum)
        {
            //std::cout << "All Tests Finished !!!\n";
            return;
        }
        std::cout << "One Sample Test !!! CurTestNum is: " << curTestNum << std::endl;
        if (mapInitialized)
        {
            if (startGoal.size() >= 2)
            {
                startGoal.clear();
            }
            int max_try = 0;
            while (startGoal.size() < 2 && max_try < 50)
            {                
                Eigen::Vector3d goal;

                //std::cout << "=========-------------- " << std::endl;
                goal(0) = rand_x_(eng_);
                goal(1) = rand_y_(eng_);
                goal(2) = rand_z_(eng_);

                if (voxelMap.query(goal) == 0)
                {   
                    //if the goal is too close to the start, ignore it
                    if (startGoal.size() == 1 && 
                        ((goal - startGoal[0]).norm() < distRange[0]))
                    {
                        continue;
                    }
                    //std::cout << "==========goal; " << goal << std::endl;
                    startGoal.emplace_back(goal);
                }
                else
                {
                    //ROS_WARN("Infeasible Position Selected !!!\n");
                    //std::cout << "goal; " << goal << std::endl;
                    
                }
                max_try += 1;

            }

            if (startGoal.size() == 2)
            {
                plan(voxelMap);
            }
        }
        return;
    }
    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_node");
    ros::NodeHandle nh_, nh_private("~");
    SFCServer planner_server_utils(nh_, nh_private);

    ros::Rate lr(1000);
    while (ros::ok())
    {
        ros::spinOnce();
        lr.sleep();
    }

    return 0;
}

