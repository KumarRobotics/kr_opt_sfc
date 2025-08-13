#ifndef SFC_COVER_OPT_HPP
#define SFC_COVER_OPT_HPP

#include <thread>
#include "sfc/sfc_utils_opt.hpp"
#include "sfc/sfc_vis.hpp"

namespace sco
{
    using namespace std;
    using namespace sfc_utils_opt;

    class OptConvexCover
    {
    private:

        //parameters
        int max_iter, max_inner_iter;
        double max_time;
        bool parallel;
        double MaxVelBox, MaxAccBox;
        double smooth_mu, epsilon, rho, progress, damping;
        int obj_type;

        //evaluation, benchmark data;
        Eigen::MatrixXd eval_data;
    
        //eval_data contains data for each iteration
        Trajectory<5> vis_traj;
        sfc_vis visualizer;
        int seg_num;

        //heristic trajectories
        Trajectory<5> jerk_traj;
        PolyhedraH hPolys, vishPolys;
        EllipsoidsR opt_ellips;

        //update during iterations
        Eigen::Matrix<double, 6, 4> bd;
        double traj_cost;
       
    public:

        bool debug;

        OptConvexCover(){}
        OptConvexCover(ros::NodeHandle &nh)
        {

            nh.getParam("MaxVelBox", MaxVelBox);
            nh.getParam("MaxAccBox", MaxAccBox);
            nh.getParam("sfc_cover_opt/max_iter", max_iter);
            nh.getParam("sfc_cover_opt/max_inner_iter", max_inner_iter);
            nh.getParam("sfc_cover_opt/max_time", max_time);
            nh.getParam("sfc_cover_opt/debug", debug);
            nh.getParam("sfc_cover_opt/parallel", parallel);

            nh.getParam("sfc_cover_opt/smooth_mu", smooth_mu);
            //control the l1 penalty function
            nh.getParam("sfc_cover_opt/epsilon", epsilon);
            nh.getParam("sfc_cover_opt/rho", rho); 
            nh.getParam("sfc_cover_opt/damping", damping);
            //control the tradeoff between the distance penality and the volume

            nh.getParam("sfc_cover_opt/obj_type", obj_type);
            nh.getParam("sfc_cover_opt/progress", progress);


            bd = Eigen::Matrix<double, 6, 4>::Zero();
            bd(0, 0) = 1.0;
            bd(1, 0) = -1.0;
            bd(2, 1) = 1.0;
            bd(3, 1) = -1.0;
            bd(4, 2) = 1.0;
            bd(5, 2) = -1.0;

        }

        inline void setVisualizer(sfc_vis &vis){visualizer = vis;}
        inline void gethPolys(PolyhedraH &plys){ plys = vishPolys;}
        inline void getTraj(Trajectory<5> &traj){traj = jerk_traj;}
        inline void getEvalData(Eigen::MatrixXd &data){data = eval_data;}

        inline void visualizeEllipsoids(const EllipsoidsR &ellips)
        {

            std::vector<Eigen::Matrix3d> Rs;
            std::vector<Eigen::Vector3d> ds;
            std::vector<Eigen::Vector3d> rs;
            for (const auto &e : ellips)
            {
                Rs.push_back(e.R);
                ds.push_back(e.d);
                rs.push_back(e.r);
            }
            visualizer.visualizeEllipsoids(Rs, ds, rs);
        }

        inline bool convexCover(const Eigen::MatrixXd &iniState,
                                const Eigen::MatrixXd &finState,
                                const std::vector<Eigen::Vector3d> &pcs,
                                const Eigen::Vector3d &lowCorner,
                                const Eigen::Vector3d &highCorner,
                                const double &range,
                                const std::vector<Eigen::Vector3d> &waypoints, // path include the start and end point
                                PolyhedraH &hPolys)
        {
            if (waypoints.size() == 2)
            {
                std::cout << "only one segment" << std::endl;
                return false;
            }

            std::cout << "waypoints size: " << waypoints.size() << std::endl;
            
            ros::Time start = ros::Time::now();
            std::vector<Eigen::Vector3d> sub_points;
            sub_points.reserve(waypoints.size() * 2);
            sub_points.push_back(waypoints[0]); 
            
            for (size_t i = 0; i < waypoints.size() - 1; ++i)
            {
                Eigen::Vector3d start = waypoints[i];
                Eigen::Vector3d end = waypoints[i + 1];
                Eigen::Vector3d diff = end - start;
                double dist = diff.norm();
            
                if (dist > progress * 1.5)  // Only subdivide if necessary
                {
                    int num_segments = std::ceil(dist / progress);  // Round UP
                    Eigen::Vector3d step = diff / num_segments;     // Evenly divide
            
                    for (int j = 1; j < num_segments; ++j) 
                    {
                        sub_points.push_back(start + step * j);
                    }
                }
                
                sub_points.push_back(end);  
            }
            

            Eigen::Map<Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> init_wps(sub_points[0].data(), 3, sub_points.size());
            Eigen::Matrix3Xd cur_wps = init_wps;

            std::cout << "sub_points size: " << sub_points.size() << std::endl;
            seg_num = sub_points.size() - 1;


            //map wps to points
            EllipsoidsR cur_ellips;
            Eigen::Matrix2Xd cur_y = Eigen::MatrixXd::Zero(2, seg_num);

            // init the ellipsoid with line segment
            initEllipsoids(cur_wps, pcs, 
                           lowCorner, highCorner,
                           range, cur_ellips);

            visualizeEllipsoids(cur_ellips);
            Eigen::VectorXd evals(4);
            std::vector<Eigen::Vector3d> eval_points;
            eval_data.resize(max_iter, 6);

            start = ros::Time::now();
            Eigen::Vector2d y_i; 

            for (int k = 0; k < max_iter; k++)
            {

                //ros::Time iter_time = ros::Time::now();
                findhPolys(cur_ellips, hPolys);
                //std::cout << "[step one]: find hpolys done" << std::endl;
       
                for (int k2 = 0; k2 < max_inner_iter; k2++)
                {
                    findOptWps(iniState, finState, hPolys, cur_y, cur_ellips, cur_wps);
                    //std::cout << "[step three]: find waypoints done" << std::endl;

                    findOptEllipsoids(cur_wps, hPolys, cur_y, cur_ellips);
                    //std::cout << "[step three]: find ellipsoids done" << std::endl;

                    validateConst(cur_wps, hPolys, cur_ellips);
                    //std::cout << "[step four]: validate constraints done" << std::endl;
                    //std::cout << "cur_wps size: " << cur_wps.cols() << std::endl;
                    for (size_t i = 0; i < cur_ellips.size(); i++)
                    {
                        for (int j = 0; j < 2; j++)
                        {
                        y_i(j) = cur_ellips[i].distMax(cur_wps.col(i + j), smooth_mu);
                        }
                        //printf("dist: %f, %f\n", y_i(0), y_i(1));
                        cur_y.col(i) += y_i;
                    }
                    //std::cout << "iter: " << k << " y: " << cur_y << std::endl;
                }

                cur_y = cur_y * damping; // shrink y each time

                if (debug)
                {
                    sfc_utils::evaluatehPolys(hPolys, iniState.col(0), finState.col(0), evals, eval_points);

                    double volume = 0.0;
                    for (size_t i = 0; i < cur_ellips.size(); i++)
                    {
                        volume += sfc_utils::getEllipsVolume(cur_ellips[i].r);
                    }
                    eval_data.row(k) << evals.transpose(), volume, traj_cost;

                    std::cout << "iter: " << k << " evals: " << evals.transpose() 
                              << " volume: " << volume 
                              << std::endl;
                    
                    visualizer.visualizePolytope(hPolys);
                    std::vector<Eigen::Vector3d> wps;
                    for (int i = 0; i < cur_wps.cols(); i++)
                    {
                        wps.push_back(cur_wps.col(i));
                    }
                    
                    visualizer.visualize(vis_traj, wps);
                    ros::Duration(1.0).sleep(); 
                    visualizeEllipsoids(cur_ellips);
                }
            }
            visualizeEllipsoids(cur_ellips);
            std::cout << "corridor generation time is " << (ros::Time::now() - start).toSec() << std::endl;
            std::vector<Eigen::Vector3d> wps;
            for (int i = 0; i < cur_wps.cols(); i++)
            {
                wps.push_back(cur_wps.col(i));
            }
            visualizer.visualize(vis_traj, wps);
            //visualizer.visualizeWaypoints(wps);
            return true;
        }

        bool validateConst(const Eigen::Matrix3Xd &wps,  
                           const PolyhedraH &hPolys,
                           const EllipsoidsR &rEllips)
        {
            //test one: check if the waypoints are inside the polytopes
            int flag = 0;
            for (size_t j = 0; j < hPolys.size(); j++)
            {   
                for (size_t i = j; i < j + 2; i++)
                {
                    if (!sfc_utils::checkInsidePoly(wps.col(i), hPolys[j], 1.0e-3))
                    {
                        flag = 1;
                    }              
                }
            }

            if (flag)
            {
                std::cout << "validateConst failed!" << std::endl;
                return false;
            }

            return true;
        }

        void findOptWps(const Eigen::MatrixXd &iniState,
                        const Eigen::MatrixXd &finState,
                        const PolyhedraH &hPolys,
                        const Eigen::Matrix2Xd &y,
                        const EllipsoidsR &rEllips,
                        Eigen::Matrix3Xd &wps)
        {
            Eigen::VectorXd times;

            traj_cost = sfc_utils_opt::getDistHeuristics(iniState.col(0), finState.col(0), 
                                            hPolys, y, rEllips, 
                                            wps, smooth_mu, rho);




            if (debug)
            {
                if (times.size() == 0)
                {
                    times = Eigen::VectorXd::Ones(wps.cols() - 1);
                }
                sfc_utils_opt::evalTraj(iniState, finState, times, wps, vis_traj);
            }
        }


        void findhPolys(const EllipsoidsR &rEllips,
                        PolyhedraH &hPolys)
        {
            hPolys.clear();
            hPolys.resize(rEllips.size());
            if (parallel)
            {
                std::vector<thread> hPoly_threads;
                for (size_t i = 0; i < rEllips.size(); i++)
                {
                    hPoly_threads.emplace_back(thread(sfc_utils_opt::findOutPolytope,
                                                        rEllips[i], 
                                                        ref(hPolys[i]),
                                                        epsilon));
                }
                for (size_t i = 0; i < rEllips.size(); i++)
                {
                    hPoly_threads[i].join();
                }
            } else {
                for (size_t i = 0; i < rEllips.size(); i++)
                {
                    sfc_utils_opt::findOutPolytope(rEllips[i], 
                                                   hPolys[i],
                                                   epsilon);
                }
            }
        }
        

        void findOptEllipsoids(const Eigen::Matrix3Xd &wps,
                               const PolyhedraH &hPolys,
                               const Eigen::Matrix2Xd &y,
                               EllipsoidsR &rEllips)
        {
            if (parallel) {
                std::vector<thread> ell_threads;
                for (size_t i = 0; i < hPolys.size(); ++i) {
                    ell_threads.emplace_back(thread(sfc_utils_opt::maxVolInsOutEllipsoid,
                                                    hPolys[i], 
                                                    wps.col(i), wps.col(i + 1),
                                                    y.col(i), ref(rEllips[i]),
                                                    smooth_mu, rho));
                }
                for (size_t i = 0; i < hPolys.size(); ++i) {
                    ell_threads[i].join();
                }
            } else {
                for (size_t i = 0; i < hPolys.size(); ++i)
                {
                    sfc_utils_opt::maxVolInsOutEllipsoid(hPolys[i], 
                                                        wps.col(i), wps.col(i + 1),
                                                        y.col(i), ref(rEllips[i]),
                                                        smooth_mu, rho);
                }
            }
        }
        

        void initEllipsoids(const Eigen::Matrix3Xd &waypoints,
                            const std::vector<Eigen::Vector3d> &points,
                            const Eigen::Vector3d &lowCorner,
                            const Eigen::Vector3d &highCorner,
                            const double &range,
                            EllipsoidsR &cur_ellips)
        {
            cur_ellips.clear();
            std::vector<Eigen::Vector3d> valid_pc;

            for (int i = 0; i < waypoints.cols() - 1; i++)
            {
                Eigen::Vector3d a = waypoints.col(i);
                Eigen::Vector3d b = waypoints.col(i + 1);

                bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
                bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
                bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
                bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
                bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));
                bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));

                valid_pc.clear();
                for (const Eigen::Vector3d &p : points)
                {
                    if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < epsilon)
                    {
                        valid_pc.emplace_back(p);
                    }
                }

                Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3, valid_pc.size());

                Eigen::Matrix3d R;
                Eigen::Vector3d d, r;

                sfc_utils::initEllipsoid(a, b, R, d, r);
                //std::cout << "------------------------------- "  << std::endl;

                EllipsoidR e(d, r, R, pc, bd);
                cur_ellips.push_back(e);           
            }

            return;
            
        }



    };


} // namespace sfc_cover_opt


#endif