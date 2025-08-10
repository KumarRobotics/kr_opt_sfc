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

    inline void getMesh2D(const Eigen::MatrixX3d &hPoly, Eigen::Matrix3Xd &curTris)
    {
        const double epsilon = 1e-6;

        // Step 1: Find an interior point (simple heuristic: centroid of polygon formed by intersection points)
        // For that, we find all intersection points of every pair of halfspaces (lines) and pick one inside all halfspaces.

        std::vector<Eigen::Vector2d> vertices;

        // Number of halfplanes
        int m = hPoly.rows();

        // Compute pairwise intersections of the lines (h0*x + h1*y + h2 = 0)
        for (int i = 0; i < m; i++)
        {
            Eigen::Vector3d h1 = hPoly.row(i);
            for (int j = i + 1; j < m; j++)
            {
                Eigen::Vector3d h2 = hPoly.row(j);

                // Solve system:
                // [h1(0) h1(1)] [x] = -h1(2)
                // [h2(0) h2(1)] [y] = -h2(2)

                Eigen::Matrix2d A;
                A << h1(0), h1(1),
                    h2(0), h2(1);

                // Check if lines are parallel (determinant close to zero)
                double det = A.determinant();
                if (std::abs(det) < epsilon)
                    continue;  // Parallel or nearly parallel

                Eigen::Vector2d b(-h1(2), -h2(2));
                Eigen::Vector2d p = A.colPivHouseholderQr().solve(b);

                // Check if point p satisfies all halfspace inequalities
                bool inside = true;
                for (int k = 0; k < m; k++)
                {
                    double val = hPoly(k, 0) * p(0) + hPoly(k, 1) * p(1) + hPoly(k, 2);
                    if (val > epsilon)
                    {
                        inside = false;
                        break;
                    }
                }

                if (inside)
                    vertices.push_back(p);
            }
        }

        if (vertices.size() < 3)
        {
            // Not enough vertices to form polygon
            curTris.resize(3, 0);
            return;
        }

        // Step 2: Compute convex hull of vertices (using QuickHull)
        quickhull::QuickHull<double> qh;
        Eigen::Matrix<double, 2, Eigen::Dynamic> vPoly(2, vertices.size());
        for (size_t i = 0; i < vertices.size(); i++)
            vPoly.col(i) = vertices[i];

        auto hull = qh.getConvexHull(vPoly.data(), vPoly.cols(), false, true);

        const auto &idxBuffer = hull.getIndexBuffer();
        int numTriangles = idxBuffer.size() / 3;

        // Step 3: Convert to 3D triangles with z=0 for visualization
        curTris.resize(3, numTriangles * 3);
        for (int i = 0; i < numTriangles * 3; i++)
        {
            curTris(0, i) = vPoly(0, idxBuffer[i]);
            curTris(1, i) = vPoly(1, idxBuffer[i]);
            curTris(2, i) = 0.0;  // z = 0 for 2D
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

    double cross(const Eigen::Vector2d& O, const Eigen::Vector2d& A, const Eigen::Vector2d& B) {
        return (A.x() - O.x()) * (B.y() - O.y()) - (A.y() - O.y()) * (B.x() - O.x());
    }

    Eigen::Matrix2Xd convexHull(const Eigen::Matrix2Xd& points) {
        int n = points.cols();
        if (n <= 1) return points;

        // Copy points to vector for easier sorting and unique operations
        std::vector<Eigen::Vector2d> pts;
        pts.reserve(n);
        for (int i = 0; i < n; i++) {
            pts.emplace_back(points(0, i), points(1, i));
        }

        // Sort lex by x, then y
        std::sort(pts.begin(), pts.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
            return (a.x() < b.x()) || (a.x() == b.x() && a.y() < b.y());
        });

        // Remove duplicates
        pts.erase(std::unique(pts.begin(), pts.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
            return a.x() == b.x() && a.y() == b.y();
        }), pts.end());

        std::vector<Eigen::Vector2d> lower, upper;

        // Build lower hull
        for (const auto& p : pts) {
            while (lower.size() >= 2 && cross(lower[lower.size() - 2], lower.back(), p) <= 0)
                lower.pop_back();
            lower.push_back(p);
        }

        // Build upper hull
        for (int i = (int)pts.size() - 1; i >= 0; i--) {
            const auto& p = pts[i];
            while (upper.size() >= 2 && cross(upper[upper.size() - 2], upper.back(), p) <= 0)
                upper.pop_back();
            upper.push_back(p);
        }

        // Remove duplicate endpoints
        lower.pop_back();
        upper.pop_back();

        // Concatenate lower and upper hull points
        lower.insert(lower.end(), upper.begin(), upper.end());

        // Convert back to Eigen::Matrix2Xd
        Eigen::Matrix2Xd hull(2, lower.size());
        for (size_t i = 0; i < lower.size(); i++) {
            hull(0, i) = lower[i].x();
            hull(1, i) = lower[i].y();
        }

        return hull;
    }



    void projectHalfspacesTo2D(const std::vector<Eigen::MatrixX4d>& hpolys,
                            std::vector<Eigen::MatrixX3d>& polygons,
                            std::vector<Eigen::Matrix2Xd>& allTris) 
    {
        polygons.clear();
        allTris.clear();

        for (const auto& hPoly : hpolys) {
            // 1. Convert H-representation to vertices (V-representation)
            Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::ColMajor> vPoly;
            geo_utils::enumerateVs(hPoly, vPoly);
            

            // 2. Project vertices onto XY plane with z = 0
            Eigen::Matrix2Xd projpts(2, vPoly.cols());
            projpts.row(0) = vPoly.row(0); // x
            projpts.row(1) = vPoly.row(1); // y
            // projpts.row(2).setZero();      // z = 0

            // // 3. Compute convex hull of projected points (3D points with z=0)
            // quickhull::QuickHull<double> qh;
            // auto hull = qh.getConvexHull(projpts.data(), projpts.cols(), false, true);

            // const auto &idBuffer = hull.getIndexBuffer();
            // const auto &vtBuffer = hull.getVertexBuffer();
            // int ids = idBuffer.size();
            // Eigen::Matrix3Xd mesh;
            // mesh.resize(3, ids);
            // quickhull::Vector3<double> v;
            // for (int i = 0; i < ids; i++)
            // {
            //     v = vtBuffer[idBuffer[i]];
            //     mesh(0, i) = v.x;
            //     mesh(1, i) = v.y;
            //     mesh(2, i) = v.z;
            // }

            Eigen::Matrix2Xd hulls = convexHull(projpts);

            allTris.push_back(hulls);

            //std::cout << "Projected 2D Mesh size is " << hulls.cols() << std::endl;

            Eigen::MatrixX3d hPoly2d(hulls.cols(), 3);
            Eigen::Vector2d normal, p1, p2;
            for (int i = 0; i < hulls.cols(); i++)
            {
                p1 = hulls.col(i);
                p2 = hulls.col((i + 1) % hulls.cols());
                //std::cout << "edge0: " << edge0.transpose() << ", edge1: " << edge1.transpose() << std::endl;

                // use p1 and p2 to form a line in 2d
                // Direction vector along the edge
                Eigen::Vector2d dir = p2 - p1;

                // Normal vector (perpendicular to dir)
                // Eigen::Vector2d normal(-dir.y(), dir.x());
                // normal.normalize(); // Keep it unit length for convenience

                normal = Eigen::Vector2d(dir.y(), -dir.x());
                normal.normalize();

                double d = -normal.dot(p1);
                //test if p2 is on this line
                // if (normal.dot(p2) + d < 1e-6)
                // {
                //     std::cout << "p2 is on the line" << std::endl;
                // }
                //std::cout << "normal: " << normal.transpose() << ", d: " << d << std::endl;

                hPoly2d.row(i).head<2>() = normal;
                hPoly2d(i, 2) = d;
            }

            polygons.push_back(hPoly2d);
        }
    }


}

#endif
