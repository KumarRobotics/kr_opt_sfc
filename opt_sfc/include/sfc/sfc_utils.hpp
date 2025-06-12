#ifndef SFC_UTILS_HPP
#define SFC_UTILS_HPP

#include "gcopter/geo_utils.hpp"  // Needed for geo_utils::enumerateVs
#include <Eigen/Eigen>
#include <cmath>        // for std::sqrt, std::atan2, std::cos, std::sin, M_PI
#include <cstdint>      
#include <vector>


namespace sfc_utils
{

    inline void chol3d(const Eigen::Matrix3d &A,
                       Eigen::Matrix3d &L)
    {
        L(0, 0) = sqrt(A(0, 0));
        L(0, 1) = 0.0;
        L(0, 2) = 0.0;
        L(1, 0) = 0.5 * (A(0, 1) + A(1, 0)) / L(0, 0);
        L(1, 1) = sqrt(A(1, 1) - L(1, 0) * L(1, 0));
        L(1, 2) = 0.0;
        L(2, 0) = 0.5 * (A(0, 2) + A(2, 0)) / L(0, 0);
        L(2, 1) = (0.5 * (A(1, 2) + A(2, 1)) - L(2, 0) * L(1, 0)) / L(1, 1);
        L(2, 2) = sqrt(A(2, 2) - L(2, 0) * L(2, 0) - L(2, 1) * L(2, 1));
        return;
    }

    inline bool smoothedL1(const double &mu,
                           const double &x,
                           double &f,
                           double &df)
    {
        if (x < 0.0)
        {
            return false;
        }
        else if (x > mu)
        {
            f = x - 0.5 * mu;
            df = 1.0;
            return true;
        }
        else
        {
            const double xdmu = x / mu;
            const double sqrxdmu = xdmu * xdmu;
            const double mumxd2 = mu - 0.5 * x;
            f = mumxd2 * sqrxdmu * xdmu;
            df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
            return true;
        }
    }

    inline void getMesh(const Eigen::MatrixX4d &hPoly,
                        Eigen::Matrix3Xd &curTris)
    {
        Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
        geo_utils::enumerateVs(hPoly, vPoly);
        quickhull::QuickHull<double> tinyQH;
        const auto polyHull = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
        const auto &idxBuffer = polyHull.getIndexBuffer();
        int hNum = idxBuffer.size() / 3;

        curTris.resize(3, hNum * 3);
        for (int i = 0; i < hNum * 3; i++)
        {
            curTris.col(i) = vPoly.col(idxBuffer[i]);
        }
    }

    inline void getMeshVolume(const Eigen::Matrix3Xd &Meshs,
                              double &volume)
    {
        //The volume of a polyhedron composed of N triangular faces 
        //with vertices (a_i,b_i,c_i) can be computed using the curl theorem 
        //as 1/6 * sum_{i=1}^{N} a_i * n_i, n_i is the normal of the i-th face
        // n_i = (b_i - a_i) x (c_i - a_i)
        volume = 0.0;
        for (int i = 0; i < Meshs.cols(); i += 3)
        {
            auto normal = (Meshs.col(i + 1) - Meshs.col(i)).cross(Meshs.col(i + 2) - Meshs.col(i));
            volume += Meshs.col(i).dot(normal);
        }
        volume /= 6.0;
    }

    inline double getEllipsVolume(const Eigen::Vector3d &r)
    {
        return 4.0 / 3.0 * M_PI * r(0) * r(1) * r(2);
    }

    inline Eigen::Matrix3d vec3_to_rotation(const Eigen::Vector3d &v) 
    {
        // zero roll
        Eigen::Vector3d rpy(0, std::atan2(-v(2), v.topRows<2>().norm()),
                    std::atan2(v(1), v(0)));
        Eigen::Quaterniond qx(cos(rpy(0) / 2), sin(rpy(0) / 2), 0, 0);
        Eigen::Quaterniond qy(cos(rpy(1) / 2), 0, sin(rpy(1) / 2), 0);
        Eigen::Quaterniond qz(cos(rpy(2) / 2), 0, 0, sin(rpy(2) / 2));
        return Eigen::Matrix3d(qz * qy * qx);
    }


    inline void initEllipsoid(const Eigen::Vector3d &a,
                              const Eigen::Vector3d &b,
                              Eigen::Matrix3d &R,
                              Eigen::Vector3d &p,
                              Eigen::Vector3d &r)
    {
        const double f = (a - b).norm() / 2;
        Eigen::Vector3d axes = Eigen::Vector3d::Constant(0.1);
        axes(0) += f;
        
        const auto Ri = vec3_to_rotation(b - a);
        R = Ri;
        p = (a + b) / 2;
        r = axes;
    }


    inline bool checkInsidePoly(const Eigen::Vector3d &pt,
                                const Eigen::MatrixX4d &hPoly,
                                const double epsilon = 1.0e-6)
    {

        const Eigen::ArrayXd hNorm = hPoly.leftCols<3>().rowwise().norm();
        Eigen::MatrixX3d A = hPoly.leftCols<3>().array().colwise() / hNorm;
        Eigen::VectorXd  b = -hPoly.rightCols<1>().array() / hNorm;

        for (int i = 0; i < hPoly.rows(); i++){
            double linear_const = A.row(i) * pt - b(i);

            if ( linear_const > epsilon)
            {
                return false;
            }
        }

        return true;
    }

    inline bool evaluatehPolys(const std::vector<Eigen::MatrixX4d> &hpolys,
                               const Eigen::Vector3d &start,
                               const Eigen::Vector3d &goal,
                               Eigen::VectorXd &evals,
                               std::vector<Eigen::Vector3d> &inner_pts)
    {
        evals.setZero();
        inner_pts.clear();
        // 0: poly number 1: overall volume 2: overlapped volume 3: distance
        evals(0) = hpolys.size();

        bool valid = true;
        inner_pts.push_back(start);
        for (size_t i = 0; i < hpolys.size(); i++)
        {
            //overall volume, overlapped volume, distance
            //compute polytope volume
            //std::cout << "hpolys[i] size is " << hpolys[i].rows() << std::endl;
            Eigen::Matrix3Xd mesh;
            getMesh(hpolys[i], mesh);
            double volume;
            getMeshVolume(mesh, volume);
            evals(1) += volume;

            //std::cout << "volume is " << volume << std::endl;
            if (i == hpolys.size() - 1)
            {
                break;
            }
            //compute overlapped volume
            Eigen::Matrix3Xd mesh2;
            Eigen::MatrixX4d hPoly_inter; //is the intersection polytope of hpolys[i] and hpolys[i+1]
            hPoly_inter.resize(hpolys[i].rows() + hpolys[i + 1].rows(), 4);
            hPoly_inter << hpolys[i], hpolys[i + 1];
            getMesh(hPoly_inter, mesh2);
            double volume2;
            getMeshVolume(mesh2, volume2);
            evals(2) += volume2;

            //compute distance
            Eigen::Vector3d inner;
            if (geo_utils::findInterior(hPoly_inter, inner))
            {
                inner_pts.emplace_back(inner);
            }
            else
            {
                ROS_WARN("No Inner Point Found !!!\n");
                valid = false;
            }

        }
        
        inner_pts.push_back(goal);
        if (inner_pts.size() > 1)
        {
            for (size_t i = 0; i < inner_pts.size() - 1; i++)
            {
                evals(3) += (inner_pts[i] - inner_pts[i + 1]).norm();
            }
        } 

        return valid;

    }

}

#endif
