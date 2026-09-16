#ifndef STAR_HPP
#define STAR_HPP
#include <algorithm>
#include <numeric>
#include <cmath>
#include <vector>
#include <iostream>
#include <cstddef>
#include <chrono>
#include <iomanip>
#include <array>
#include <utility>
#include <random>
#include <thread>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <unordered_set>

#include "gcopter/geo_utils.hpp"
#include "gcopter/quickhull.hpp"
#include "gcopter/firi.hpp"
#include "sfc/sfc_utils.hpp"

#include "gcopter/geo_utils.hpp"
#include "gcopter/firi.hpp"

namespace Star
{

    struct Triangle
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Triangle() = default;
        Triangle(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c) : _a(a), _b(b), _c(c) {}
        Eigen::Vector3d _a, _b, _c;
        Eigen::Vector3d _normal;

        void calcuNormal()
        {
            _normal = (_b - _c).cross(_a - _c);
            const double nn = _normal.norm();
            if (nn > 1e-12)
                _normal /= nn;

            if (_normal.dot(_c) > 0.0)
                _normal = -_normal; // equivalent sign test
        }
    };

    using TriangleVec = std::vector<Triangle, Eigen::aligned_allocator<Triangle>>;

    struct StarNode
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        StarNode() = default;

        StarNode(const TriangleVec &mesh,
                const Eigen::MatrixX4d &kernel,
                const Eigen::Matrix<double, 6, 4> &bd,
                const Eigen::Vector3d &center)
            : star_mesh(mesh),
            star_kernel(kernel),
            bd(bd),
            center(center)
        {
            // initialize per-face belief
            const int m = static_cast<int>(star_mesh.size());
            belief = Eigen::VectorXd::Zero(m);  
        }

        // ---------- Geometry ----------
        TriangleVec star_mesh;           // star hull triangles (global frame)

        // Each row is defined by h0, h1, h2, h3 as
        // h0*x + h1*y + h2*z + h3 <= 0
        Eigen::MatrixX4d star_kernel;    // kernel constraints (global frame)
        Eigen::MatrixX4d reverse_convex_poly; // reverse convex poly (query-offset/local frame)


        Eigen::Matrix3Xd unique_sconvex_vertices; // star vertices (query-offset/local frame)
        Eigen::Matrix<double, 6, 4> bd;  // map boundary (global frame AABB planes)
        Eigen::Vector3d center; // initial query point (global frame)
        // ---------- Probabilistic state ----------
        Eigen::VectorXd belief;          // per-face  L_i


    };

    class Corridor
    {
    public:
        struct FiriTiming
        {
            double star_filter_ms = 0.0;
            double run_total_ms = 0.0;
            double tangent_total_ms = 0.0;
            double tangent_avg_ms = 0.0;
            double mvie_total_ms = 0.0;
            double mvie_avg_ms = 0.0;
            double inflation_ms = 0.0;
            int loops = 0;
        };

        enum class FiriInitMode
        {
            Constant = 0,
            Thin = 1,
            Sphere = 2
        };

        Corridor(float radius, float precision, float safety_margin)
            : _r(radius),
              _precision_qh(precision),
              _safety_margin(std::max(0.0f, safety_margin))
        {}
        Corridor() = default;
        ~Corridor() = default;

        const FiriTiming &lastTiming() const
        {
            return last_timing_;
        }

        bool hasTiming() const
        {
            return timing_valid_;
        }

        void init(float radius, float precision)
        {
            _r = radius;
            _precision_qh = precision;
        }

        void init(float radius, float precision, float safety_margin)
        {
            _r = radius;
            _precision_qh = precision;
            _safety_margin = std::max(0.0f, safety_margin);
        }

        void setSafetyMargin(float safety_margin)
        {
            _safety_margin = std::max(0.0f, safety_margin);
        }

        void flipMap(Eigen::Vector3d &point, const Eigen::Vector3d &query_point) const
        {
            const Eigen::Vector3d local_point = point - query_point;
            const double norm = local_point.norm();
            if (norm > 1e-12)
            {
                const double scale = (_r + 1.0 * (_r - norm)) / norm;
                point = query_point + scale * local_point;
            }
        }




        // query_point is global; prep_pc is shifted to local (query-offset) before hull.
        inline void createNbd(const Eigen::Vector3d &query_point,
                            const Eigen::Matrix3Xd &pc,
                            const Eigen::Matrix<double, 6, 4> &bd,
                            StarNode &star_node)
        {
            std::cout << "[createNbd] Building new star mesh at query point: "   << query_point.transpose() << std::endl;
            //////////////////// point cloud preparation ////////////////////
            // Generate bounding samples
            Eigen::Matrix3Xd bounding_samples;
            forEachBoundingFaceSampleRadial(bd, bounding_samples, 200, 1.2);

            Eigen::Matrix3Xd prep_pc(3, pc.cols() + bounding_samples.cols());
            prep_pc << pc, bounding_samples;
            //////////////////// end of point cloud preparation ////////////////////
            prep_pc.colwise() -= query_point;
            if (_safety_margin > 0.0f)
            {
                applySafetyMarginToRawclouds(prep_pc, _safety_margin);
            }
            buildNewStarNode(prep_pc, query_point, bd, star_node);
        }

        // points are in global frame; internally shifted by star_node.center to local.
        inline void updateStarNode(const Eigen::Matrix3Xd &points,
                                   StarNode &star_node)
        {
            //get all points, flip them, compute new convex hull
            Eigen::Matrix3Xd star_convex_vertices = star_node.unique_sconvex_vertices;
            Eigen::Matrix3Xd bounding_samples;
            forEachBoundingFaceSampleRadial(star_node.bd, bounding_samples, 200, 1.2);
            bounding_samples.colwise() -= star_node.center;

            Eigen::Matrix3Xd prep_pc(3, points.cols() + star_convex_vertices.cols() + bounding_samples.cols());

            prep_pc.leftCols(star_convex_vertices.cols()) = star_convex_vertices;
            Eigen::Matrix3Xd new_points_local = points.colwise() - star_node.center;
            if (_safety_margin > 0.0f)
            {
                applySafetyMarginToRawclouds(new_points_local, _safety_margin);
            }
            prep_pc.middleCols(star_convex_vertices.cols(), points.cols()) = new_points_local;
            prep_pc.rightCols(bounding_samples.cols()) = bounding_samples;

            //step one: get new star convex hull
            getStarConvexHull(prep_pc, star_node, 1e-2f);

            //reset belief
            const int m = static_cast<int>(star_node.star_mesh.size());
            star_node.belief.resize(m);
            star_node.belief.setZero();
        }

        // // a, b, pc are in global frame; internally shifted to local around query_point.
        inline void singleNbd(const Eigen::Vector3d &a,
                            const Eigen::Vector3d &b,
                            const Eigen::Matrix3Xd &pc,
                            const Eigen::Matrix<double, 6, 4> &bd,
                            StarNode &star_node)
        {
            const Eigen::Vector3d query_point = (a + b) * 0.5;

            createNbd(query_point, pc, bd, star_node);
        }

        inline void computeTangentPlanesVectorized(
            const Eigen::Matrix3Xd &forwardPC,
            const Eigen::Vector3d &fwd_a,
            const Eigen::Vector3d &fwd_b,
            Eigen::MatrixX4d &tangents,
            Eigen::RowVectorXd &distRs,
            double epsilon)
        {
            const int N = forwardPC.cols();
            
            // Vectorized distance computation
            distRs = forwardPC.colwise().norm();
            
            // Vectorized initial normals
            tangents.leftCols<3>() = (forwardPC.array().rowwise() / distRs.array()).transpose();
            tangents.col(3) = -distRs.transpose();
            
            // Vectorized dot products for all points at once
            Eigen::VectorXd dots_a = tangents.leftCols<3>() * fwd_a + tangents.col(3);
            Eigen::VectorXd dots_b = tangents.leftCols<3>() * fwd_b + tangents.col(3);
            
            // Only process problematic points (skip majority that are already correct)
            for (int k = 0; k < N; ++k)
            {
                if (dots_a(k) <= epsilon && dots_b(k) <= epsilon) continue;
                
                Eigen::Vector3d normal = tangents.block<1, 3>(k, 0).transpose();
                double dist = distRs(k);
                const Eigen::Vector3d &pt = forwardPC.col(k);
                
                if (dots_a(k) > epsilon)
                {
                    const Eigen::Vector3d delta = pt - fwd_a;
                    const double inv_delta_sq = 1.0 / delta.squaredNorm();
                    normal = fwd_a - (delta.dot(fwd_a) * inv_delta_sq) * delta;
                    const double inv_norm = 1.0 / normal.norm();
                    normal *= inv_norm;
                    dist = 1.0 / inv_norm;
                }
                
                const double dot_b = normal.dot(fwd_b) - dist;
                if (dot_b > epsilon)
                {
                    const Eigen::Vector3d delta = pt - fwd_b;
                    const double inv_delta_sq = 1.0 / delta.squaredNorm();
                    normal = fwd_b - (delta.dot(fwd_b) * inv_delta_sq) * delta;
                    const double inv_norm = 1.0 / normal.norm();
                    normal *= inv_norm;
                    dist = 1.0 / inv_norm;
                }
                
                if (normal.dot(fwd_a) - dist > epsilon)
                {
                    normal = (fwd_a - pt).cross(fwd_b - pt);
                    const double inv_norm = 1.0 / normal.norm();
                    normal *= inv_norm;
                    dist = -normal.dot(fwd_a);
                    if (dist > 0.0)
                    {
                        normal = -normal;
                        dist = -dist;
                    }
                }
                
                tangents.block<1, 3>(k, 0) = normal.transpose();
                tangents(k, 3) = -dist;
            }
        }


        // Greedy tangent filter - selects planes one by one based on minimum distance
        inline void greedyFilterTangents(
            const Eigen::Matrix3Xd &forwardPC,
            const Eigen::MatrixX3d *forwardB,      // nullptr to skip boundary constraints
            const Eigen::VectorXd *forwardD,       // nullptr to skip boundary constraints
            const Eigen::VectorXd *distDs,         // nullptr to skip boundary constraints
            const Eigen::MatrixX4d &tangents,
            const Eigen::RowVectorXd &distRs,
            Eigen::MatrixX4d &filtered,
            const double epsilon = 1.0e-6)
        {
            const int M = (forwardB != nullptr) ? forwardB->rows() : 0;
            const int N = tangents.rows();

            if (N == 0 && M == 0)
            {
                filtered.resize(0, 4);
                return;
            }

            std::vector<int> active_bd(M);
            std::iota(active_bd.begin(), active_bd.end(), 0);
            std::vector<int> active_pc(N);
            std::iota(active_pc.begin(), active_pc.end(), 0);

            std::vector<Eigen::Vector4d> kept_planes;
            kept_planes.reserve(M + N);

            // Boundary constraints are never excluded by a newly added plane (only
            // removed once directly selected), so their min only needs recomputing
            // when active_bd itself shrinks.
            const auto findMinBd = [&](int &out_id) -> double
            {
                double best = INFINITY;
                for (const int idx : active_bd)
                {
                    if ((*distDs)(idx) < best)
                    {
                        best = (*distDs)(idx);
                        out_id = idx;
                    }
                }
                return best;
            };
            const auto findMinPc = [&](int &out_id) -> double
            {
                double best = INFINITY;
                for (const int idx : active_pc)
                {
                    if (distRs(idx) < best)
                    {
                        best = distRs(idx);
                        out_id = idx;
                    }
                }
                return best;
            };

            int bdMinId = -1, pcMinId = -1;
            double minSqrD = active_bd.empty() ? INFINITY : findMinBd(bdMinId);
            double minSqrR = active_pc.empty() ? INFINITY : findMinPc(pcMinId);

            while (!active_bd.empty() || !active_pc.empty())
            {
                Eigen::RowVector3d plane_normal;
                double plane_offset;
                bool picked_bd = false;

                if (minSqrD < minSqrR)
                {
                    // Add boundary constraint
                    Eigen::Vector4d plane;
                    plane.head<3>() = forwardB->row(bdMinId).transpose();
                    plane(3) = (*forwardD)(bdMinId);
                    kept_planes.push_back(plane);

                    plane_normal = plane.head<3>().transpose();
                    plane_offset = plane(3);
                    active_bd.erase(std::find(active_bd.begin(), active_bd.end(), bdMinId));
                    picked_bd = true;
                }
                else
                {
                    // Add tangent constraint
                    kept_planes.push_back(tangents.row(pcMinId).transpose());
                    plane_normal = tangents.block<1, 3>(pcMinId, 0);
                    plane_offset = tangents(pcMinId, 3);
                    picked_bd = false;
                }

                // Compact active_pc in place: drop the just-picked point (if it came
                // from here) and every point the new plane now excludes.
                size_t write = 0;
                for (const int idx : active_pc)
                {
                    if (!picked_bd && idx == pcMinId)
                    {
                        continue;
                    }
                    const double signed_dist = plane_normal.dot(forwardPC.col(idx)) + plane_offset;
                    if (signed_dist <= -epsilon)
                    {
                        active_pc[write++] = idx;
                    }
                }
                active_pc.resize(write);

                minSqrD = active_bd.empty() ? INFINITY : findMinBd(bdMinId);
                minSqrR = active_pc.empty() ? INFINITY : findMinPc(pcMinId);
            }

            // Convert to matrix
            filtered.resize(kept_planes.size(), 4);
            for (size_t i = 0; i < kept_planes.size(); ++i)
            {
                filtered.row(i) = kept_planes[i].transpose();
            }
        }

        // Helper function for the inflation part
        inline void computeInflationPolytope(
            const Eigen::Matrix3d &R,
            const Eigen::Vector3d &p,
            const Eigen::Vector3d &r,
            const Eigen::Vector3d &a,
            const Eigen::Vector3d &b,
            const Eigen::Matrix3Xd &pc,
            const Eigen::MatrixX4d &bd,
            Eigen::MatrixX4d &hPoly,
            const double epsilon = 1.0e-6)
        {
            const auto t_start = std::chrono::steady_clock::now();
            const int N = pc.cols();
            
            // Compute transformations
            const Eigen::Vector3d r_inv = r.cwiseInverse();
            const Eigen::Matrix3d forward = r_inv.asDiagonal() * R.transpose();
            const Eigen::Matrix3d backward = R * r.asDiagonal();
            
            // Transform boundaries
            const Eigen::MatrixX3d forwardB = bd.leftCols<3>() * backward;
            const Eigen::VectorXd forwardD = bd.rightCols<1>() + bd.leftCols<3>() * p;
            
            // Transform point cloud and line segment
            const Eigen::Matrix3Xd forwardPC = forward * (pc.colwise() - p);
            const Eigen::Vector3d fwd_a = forward * (a - p);
            const Eigen::Vector3d fwd_b = forward * (b - p);
            
            // Compute distances
            const Eigen::VectorXd distDs = forwardD.cwiseAbs().cwiseQuotient(forwardB.rowwise().norm());
            
            // Compute tangent planes
            Eigen::MatrixX4d tangents(N, 4);
            Eigen::RowVectorXd distRs(N);
            computeTangentPlanesVectorized(forwardPC, fwd_a, fwd_b, tangents, distRs, epsilon);
            
            // Build polytope using greedy filter (with boundary constraints)
            Eigen::MatrixX4d forwardH;
            greedyFilterTangents(forwardPC, &forwardB, &forwardD, &distDs, 
                                tangents, distRs, forwardH, epsilon);
            
            // Transform back to original coordinate system
            const int nH = forwardH.rows();
            hPoly.resize(nH, 4);
            for (int i = 0; i < nH; ++i)
            {
                const Eigen::RowVector3d h_forward = forwardH.block<1, 3>(i, 0) * forward;
                hPoly.block<1, 3>(i, 0) = h_forward;
                hPoly(i, 3) = forwardH(i, 3) - h_forward.dot(p);
            }
            const double t_total_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_start).count();
            last_timing_.inflation_ms = t_total_ms;
            std::cout << std::fixed << std::setprecision(6)
                      << "[computeInflationPolytope] total: " << t_total_ms << " ms"
                      << std::endl;
        }



        inline void runFilteredInflation(const Eigen::Vector3d &a,
                                        const Eigen::Vector3d &b,
                                        const Eigen::Matrix3Xd &filtered_pc,
                                        const Eigen::MatrixX4d &bd,
                                        Eigen::MatrixX4d &hPoly,
                                        Eigen::Matrix3d &R,
                                        Eigen::Vector3d &p,
                                        Eigen::Vector3d &r,
                                        int iterations,
                                        double epsilon,
                                        double tolerance,
                                        bool use_filter = true,
                                        bool use_greedy = false)
        {
            const auto t_start = std::chrono::steady_clock::now();
            const int M = bd.rows();
            const int N = filtered_pc.cols();
            
            // Pre-allocate reusable buffers
            Eigen::MatrixX4d tangents(N, 4);
            Eigen::RowVectorXd distRs(N);
            Eigen::Matrix3Xd forwardPC(3, N);
            Eigen::Vector3d fwd_a, fwd_b;
            Eigen::Matrix3d forward;
            Eigen::MatrixX4d activeConstraints;
            Eigen::MatrixX4d forwardHFiltered;
            hPoly.resize(N + M, 4);
            
            double prev_det = 0.0;
            int loops_done = 0;
            double t_tangent_ms = 0.0;
            double t_mvie_ms = 0.0;
            const auto ellipsoidDet = [](const Eigen::Vector3d &r_in) {
                const double prod_r = r_in(0) * r_in(1) * r_in(2);
                return prod_r * prod_r;
            };

            for (int loop = 0; loop < iterations - 1; ++loop)
            {
                ++loops_done;
                // Transform to normalized space
                const Eigen::Vector3d r_inv = r.cwiseInverse();
                forward.noalias() = r_inv.asDiagonal() * R.transpose();
                
                forwardPC.noalias() = forward * (filtered_pc.colwise() - p);
                fwd_a.noalias() = forward * (a - p);
                fwd_b.noalias() = forward * (b - p);

                // Compute tangent planes
                const auto t_tangent_start = std::chrono::steady_clock::now();
                computeTangentPlanesVectorized(forwardPC, fwd_a, fwd_b, tangents, distRs, epsilon);
                t_tangent_ms += std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t_tangent_start).count();

                // Apply greedy filtering if requested (without boundary constraints)
                if (use_greedy && N > 0)
                {
                    greedyFilterTangents(forwardPC, nullptr, nullptr, nullptr,
                                        tangents, distRs, activeConstraints, epsilon);
                }
                else
                {
                    activeConstraints = tangents;
                }

                // Apply redundancy filtering if requested
                if (use_filter && N > 40)
                {
                    filterH(activeConstraints, forwardHFiltered, epsilon);
                }
                else
                {
                    forwardHFiltered = activeConstraints;
                }

                // Transform back to original coordinate system
                const int nH = forwardHFiltered.rows();
                const int totalH = nH + M;
                hPoly.resize(totalH, 4);
                
                for (int i = 0; i < nH; ++i)
                {
                    const Eigen::RowVector3d h_forward = forwardHFiltered.block<1, 3>(i, 0) * forward;
                    hPoly.block<1, 3>(i, 0) = h_forward;
                    hPoly(i, 3) = forwardHFiltered(i, 3) - h_forward.dot(p);
                }
                hPoly.bottomRows(M) = bd;

                // Compute maximum volume inscribed ellipsoid
                const auto t_mvie_start = std::chrono::steady_clock::now();
                firi::maxVolInsEllipsoid(hPoly, R, p, r);
                t_mvie_ms += std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t_mvie_start).count();

                // Check convergence
                const double det = ellipsoidDet(r);
                if (prev_det > 0.0)
                {
                    const double growth = (det - prev_det) / prev_det;
                    if (growth >= 0.0 && growth < tolerance)
                    {
                        break;
                    }
                }
                prev_det = det;
            }
            const double t_total_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_start).count();
            const double avg_tangent_ms = loops_done > 0 ? (t_tangent_ms / loops_done) : 0.0;
            const double avg_mvie_ms = loops_done > 0 ? (t_mvie_ms / loops_done) : 0.0;
            last_timing_.run_total_ms = t_total_ms;
            last_timing_.tangent_total_ms = t_tangent_ms;
            last_timing_.tangent_avg_ms = avg_tangent_ms;
            last_timing_.mvie_total_ms = t_mvie_ms;
            last_timing_.mvie_avg_ms = avg_mvie_ms;
            last_timing_.loops = loops_done;
            std::cout << std::fixed << std::setprecision(6)
                      << "[runFilteredInflation] total: " << t_total_ms << " ms, "
                      << "computeTangentPlanesVectorized total/avg: " << t_tangent_ms
                      << " / " << avg_tangent_ms << " ms, "
                      << "maxVolInsEllipsoid total/avg: " << t_mvie_ms
                      << " / " << avg_mvie_ms << " ms"
                      << std::endl;
        }

        // Refactored main function
        inline bool firiFiltered(const Eigen::Vector3d &a,
                                const Eigen::Vector3d &b,
                                const Eigen::Matrix3Xd &pc,
                                const Eigen::MatrixX4d &bd,
                                Eigen::MatrixX4d &hPoly,
                                Eigen::Matrix3d &R,
                                Eigen::Vector3d &p,
                                Eigen::Vector3d &r,
                                const int iterations = 5,
                                const double epsilon = 1.0e-6,
                                const double tolerance = 0.02,
                                const FiriInitMode init_mode = FiriInitMode::Sphere,
                                const double thin_radius = 0.1)
        {
            p = 0.5 * (a + b);
            timing_valid_ = true;
            last_timing_ = FiriTiming{};
            if (init_mode == FiriInitMode::Thin)
            {
                const Eigen::Vector3d ab = b - a;
                const double ab_len = ab.norm();
                R = sfc_utils::vec3_to_rotation(ab);
                r << (0.5 * ab_len + thin_radius), thin_radius, thin_radius;
            }
            else if (init_mode == FiriInitMode::Sphere)
            {
                R = Eigen::Matrix3d::Identity();
                r = Eigen::Vector3d::Ones();
            }
            else // Large
            {
                const double f = 0.5 * (a - b).norm();
                R = sfc_utils::vec3_to_rotation(b - a);
                r = Eigen::Vector3d::Constant(f);

            }
            
            Eigen::Matrix3Xd filtered_pc;
            const auto t_filter_start = std::chrono::steady_clock::now();
            starFilter(p, pc, filtered_pc);
            const double t_filter_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_filter_start).count();
            last_timing_.star_filter_ms = t_filter_ms;
            std::cout << std::fixed << std::setprecision(6)
                      << "[starFilter] total: " << t_filter_ms << " ms"
                      << std::endl;
            // const int M = bd.rows();
            // const int N = filtered_pc.cols();
            // std::cout << "[firiFiltered] Starting FIRI with " << N << " points and " << M << " boundaries." << std::endl;
            runFilteredInflation(a, b, filtered_pc, bd, hPoly, R, p, r, iterations, epsilon, tolerance);

            computeInflationPolytope(R, p, r, a, b, pc, bd, hPoly, epsilon);
            return true;
        }


        inline void starFilter(const Eigen::Vector3d &query_point,
                                const Eigen::Matrix3Xd &pc,
                                Eigen::Matrix3Xd &unique_sconvex_vertices)
        {
            // append query_point itself so its flipped (near-origin) column bounds the hull
            Eigen::Matrix3Xd prep_pc(3, pc.cols()+1);
            prep_pc << pc, query_point;

            Eigen::Matrix3Xf flip_map = prep_pc.cast<float>();
            flip_map.colwise() -= query_point.cast<float>();
            flipMap(flip_map);

            const Eigen::Index pc_cols = pc.cols();

            quickhull::QuickHull<float> qh;
            auto hull = qh.getConvexHull(flip_map.data(),
                                         flip_map.cols(),
                                         false,
                                         true,
                                         _precision_qh);

            const auto &idxBuffer = hull.getIndexBuffer();
            const Eigen::Index n = static_cast<Eigen::Index>(idxBuffer.size());

            // Pre-allocate with exact size to avoid reallocation
            std::vector<Eigen::Index> unique_idx;
            unique_idx.reserve(n);

            // Use unordered_set for O(1) lookups instead of vector<bool>
            std::unordered_set<Eigen::Index> seen;
            seen.reserve(n);

            for (Eigen::Index i = 0; i < n; ++i) {
                const Eigen::Index idx = static_cast<Eigen::Index>(idxBuffer[i]);
                if (idx >= pc_cols) {
                    continue;
                }
                if (seen.insert(idx).second) {
                    unique_idx.push_back(idx);
                }
            }
            // Direct initialization instead of loop
            unique_sconvex_vertices.resize(3, static_cast<Eigen::Index>(unique_idx.size()));
            for (size_t i = 0; i < unique_idx.size(); ++i) {
                unique_sconvex_vertices.col(i) = prep_pc.col(unique_idx[i]);
            }        
        }



        // Compute inner polytope using: 0 findPolytope, 1 findPlanes, 2 findPlanes optimize.
        bool findInnerPoly(const StarNode &star_node,
                           const Eigen::Vector3d &pa,
                           const Eigen::Vector3d &pb,
                           Eigen::MatrixX4d &hpoly,
                           int method)
        {
            if (star_node.star_mesh.empty())
            {
                hpoly.resize(0, 4);
                return false;
            }

            Eigen::MatrixX4d hplanes;
            switch (method)
            {
                case 0:
                {
                    if (!findPolytope(star_node.star_mesh,
                                      pa,
                                      pb,
                                      star_node.bd,
                                      hplanes))
                    {
                        hpoly.resize(0, 4);
                        return false;
                    }
                    filterH(hplanes, hpoly);
                    return hpoly.rows() > 0;
                }
                
                case 1:
                {
                    findPlanes(star_node.star_mesh,
                               pa,
                               pb,
                               star_node.bd,
                               hplanes);
                    break;
                }

                case 2:
                {
                    findPlanes(star_node.star_mesh,
                               pa,
                               pb,
                               star_node.bd,
                               hplanes,
                                 true);
                    break;
                }
                default:
                {
                    hpoly.resize(0, 4);
                    return false;
                }
            }
            filterH(hplanes, hpoly);

            if (hpoly.rows() == 0)
            {
                return false;
            }

            return true;
        }


        void convexHull(const Eigen::Matrix3Xd &points,
                        Eigen::Matrix3Xd &vertices,
                        float precision = 0.01f)
        {

            quickhull::QuickHull<double> qh;
            auto hull = qh.getConvexHull(points.derived().data(),
                                         points.derived().cols(),
                                         false,
                                         true,
                                         precision);
            const auto &idxBuffer = hull.getIndexBuffer();

            int hNum3 = idxBuffer.size();
            vertices.resize(3, hNum3);
            for (int i = 0; i < hNum3; i++)
            {
                vertices.col(i) = points.derived().col(idxBuffer[i]);
            }
        }


        // rawclouds are in local (query-offset) frame; outputs mesh/kernel in global frame.
        void getStarConvexHull(const Eigen::Matrix3Xd &rawclouds, // centered at query point
                               StarNode &star_node,
                               float precision = 0.01f)
        {
            /***************** I: Flip-space convex hull *****************/
            // convert to float
            Eigen::Matrix3Xf prep_pc = rawclouds.cast<float>();
            flipMap(prep_pc);

            quickhull::QuickHull<float> qh;
            auto hull = qh.getConvexHull(prep_pc.data(),
                                         prep_pc.cols(),
                                         false,
                                         true,
                                         precision);

            const auto &idxBuffer = hull.getIndexBuffer();
            const Eigen::Index n = static_cast<Eigen::Index>(idxBuffer.size());
            Eigen::Matrix3Xd sconvex_vertices(3, n); // local frame (query offset removed)

            Eigen::Matrix3Xd convex_vertices(3, n); // local frame (query offset removed)
            for (Eigen::Index i = 0; i < n; ++i)
            {
                sconvex_vertices.col(i) = rawclouds.col(idxBuffer[i]);
                convex_vertices.col(i) = prep_pc.col(idxBuffer[i]).cast<double>();
            }
                
            //original convex hull
            hPolyFromMatrixWithOffset(convex_vertices,
                                      star_node.center,
                                      star_node.reverse_convex_poly);


            /***************** II: store unique vertices for future refinements *****************/
            std::vector<bool> seen(rawclouds.cols(), false);
            std::vector<Eigen::Index> unique_idx;
            unique_idx.reserve(n);

            for (Eigen::Index i = 0; i < n; ++i) {
                if (!seen[idxBuffer[i]]) {
                    seen[idxBuffer[i]] = true;
                    unique_idx.push_back(idxBuffer[i]);
                }
            }


            Eigen::Matrix3Xd unique_sconvex_vertices(3, static_cast<Eigen::Index>(unique_idx.size()));
            for (size_t i = 0; i < unique_idx.size(); ++i)
                unique_sconvex_vertices.col(i) = rawclouds.col(unique_idx[i]);
            star_node.unique_sconvex_vertices = unique_sconvex_vertices;

            /***************** III: update global star mesh and kernel *****************/

            //step one: update star mesh (avoid extra global matrix allocation)
            star_node.star_mesh = trianglesFromMatrixWithOffset(sconvex_vertices, star_node.center);

            //step two: update star kernel from the stored mesh
            Eigen::MatrixX4d star_kernel_raw;
            hpolyFromMesh(star_node.star_mesh, star_node.center, star_kernel_raw);

            filterH(star_kernel_raw, star_node.star_kernel);

            //std::cout << "[Star] Filtered star kernel has " << star_node.star_kernel.rows() << " half-spaces." << std::endl;
        }


    private:
        float _r = 5.0f, _precision_qh = 0.05f, _safety_margin = 0.0f;
        FiriTiming last_timing_{};
        bool timing_valid_ = false;

        static Triangle makeTriangle(const Eigen::Vector3d &a,
                                     const Eigen::Vector3d &b,
                                     const Eigen::Vector3d &c)
        {
            Triangle tri(a, b, c);
            tri.calcuNormal();
            return tri;
        }

        static TriangleVec trianglesFromMatrixWithOffset(const Eigen::Matrix3Xd &tri_vertices_local,
                                                         const Eigen::Vector3d &offset)
        {
            TriangleVec triangles;
            const Eigen::Index tri_count = tri_vertices_local.cols() / 3;
            if (tri_count <= 0)
            {
                return triangles;
            }
            triangles.reserve(static_cast<std::size_t>(tri_count));
            for (Eigen::Index i = 0; i < tri_count; ++i)
            {
                triangles.emplace_back(makeTriangle(tri_vertices_local.col(i * 3) + offset,
                                                    tri_vertices_local.col(i * 3 + 1) + offset,
                                                    tri_vertices_local.col(i * 3 + 2) + offset));
            }
            return triangles;
        }

        static void hPolyFromMatrixWithOffset(const Eigen::Matrix3Xd &tri_vertices_local,
                                              const Eigen::Vector3d &offset,
                                              Eigen::MatrixX4d &hpoly)
        {
            const TriangleVec triangles = trianglesFromMatrixWithOffset(tri_vertices_local, offset);
            hpolyFromMesh(triangles, offset, hpoly);
        }

        static Eigen::Matrix3Xd meshVerticesLocal(const TriangleVec &triangles,
                                                  const Eigen::Vector3d &center)
        {
            const Eigen::Index tri_count = static_cast<Eigen::Index>(triangles.size());
            Eigen::Matrix3Xd verts(3, tri_count * 3);
            for (Eigen::Index i = 0; i < tri_count; ++i)
            {
                const Triangle &tri = triangles[static_cast<size_t>(i)];
                verts.col(i * 3) = tri._a - center;
                verts.col(i * 3 + 1) = tri._b - center;
                verts.col(i * 3 + 2) = tri._c - center;
            }
            return verts;
        }

        bool buildConvexVertices(const TriangleVec &star_mesh,
                                 const Eigen::Vector3d &query_point,
                                 Eigen::Matrix3Xd &star_convex_vertices,
                                 Eigen::Matrix3Xd &convex_vertices)
        {
            star_convex_vertices = meshVerticesLocal(star_mesh, query_point);
            if (star_convex_vertices.cols() < 3)
            {
                return false;
            }
            convexHull(star_convex_vertices, convex_vertices);
            return convex_vertices.cols() >= 3;
        }

        static void extractTriangleMatrices(const Eigen::Matrix3Xd &convex_vertices,
                                            Eigen::Matrix3Xd &A,
                                            Eigen::Matrix3Xd &B,
                                            Eigen::Matrix3Xd &C)
        {
            const size_t ori_m = static_cast<size_t>(convex_vertices.cols() / 3);
            A.resize(3, static_cast<Eigen::Index>(ori_m));
            B.resize(3, static_cast<Eigen::Index>(ori_m));
            C.resize(3, static_cast<Eigen::Index>(ori_m));
            for (size_t j = 0; j < ori_m; ++j)
            {
                A.col(static_cast<Eigen::Index>(j)) = convex_vertices.col(static_cast<Eigen::Index>(j * 3));
                B.col(static_cast<Eigen::Index>(j)) = convex_vertices.col(static_cast<Eigen::Index>(j * 3 + 1));
                C.col(static_cast<Eigen::Index>(j)) = convex_vertices.col(static_cast<Eigen::Index>(j * 3 + 2));
            }
        }

        static void applySafetyMarginToRawclouds(Eigen::Matrix3Xd &rawclouds,
                                                 double margin)
        {
            if (margin <= 0.0 || rawclouds.cols() == 0)
            {
                return;
            }

            const Eigen::Index n = rawclouds.cols();
            for (Eigen::Index i = 0; i < n; ++i)
            {
                const double norm = rawclouds.col(i).norm();
                if (norm <= 1e-12)
                {
                    continue;
                }
                const double scaled = std::max(0.0, norm - margin) / norm;
                rawclouds.col(i) *= scaled;
            }
        }

        static Eigen::Vector3d orientedNormal(const Triangle &tri,
                                              const Eigen::Vector3d &query_point)
        {
            Eigen::Vector3d normal = (tri._b - tri._c).cross(tri._a - tri._c);
            const double nn = normal.norm();
            if (nn > 1e-12)
            {
                normal /= nn;
            }
            if (normal.dot(query_point - tri._a) > 0.0)
            {
                normal = -normal;
            }
            return normal;
        }

        // triangles are in global frame; query_point orients normals in global frame.
        static void hpolyFromMesh(const TriangleVec &triangles,
                                  const Eigen::Vector3d &query_point,
                                  Eigen::MatrixX4d &hpoly)
        {
            const Eigen::Index tri_count = static_cast<Eigen::Index>(triangles.size());
            hpoly.resize(tri_count, 4);
            for (Eigen::Index i = 0; i < tri_count; ++i)
            {
                const Triangle &tri = triangles[static_cast<size_t>(i)];
                const Eigen::Vector3d normal = orientedNormal(tri, query_point);
                hpoly.row(i).head<3>() = normal.transpose();
                hpoly(i, 3) = -normal.dot(tri._a);
            }
        }

        // flipMap expects local (query-offset) points around the origin.
        void flipMap(Eigen::Matrix3Xf &input) const
        {
            Eigen::RowVectorXf norms = input.colwise().norm();
            // guard columns at (or near) the origin to avoid
            // a divide-by-zero that would otherwise inject NaN into the hull input.
            Eigen::RowVectorXf safe_norms = norms.cwiseMax(1e-6f);
            Eigen::RowVectorXf scales = (_r + 1.0f * (_r - safe_norms.array())) / safe_norms.array();
            for (Eigen::Index i = 0; i < norms.size(); ++i)
            {
                if (norms(i) <= 1e-6f)
                {
                    scales(i) = 1.0f;
                }
            }
            input.array().rowwise() *= scales.array();
        }

        // rawclouds are in local (query-offset) frame; query_point is global.
        void buildNewStarNode(const Eigen::Matrix3Xd &rawclouds,
                           const Eigen::Vector3d &query_point,
                           const Eigen::Matrix<double, 6, 4> &bd,
                           StarNode &star_node)
        { 
            //initialize star_node
            star_node = StarNode();
            star_node.bd = bd;
            star_node.center = query_point;

            getStarConvexHull(rawclouds,
                              star_node,
                             _precision_qh);

            const int m = static_cast<int>(star_node.star_mesh.size());
            star_node.belief.resize(m);
            star_node.belief.setZero();
        }

        // star_mesh is in global frame; query_point is global.
        void findPlanes(const TriangleVec &star_mesh,
                        const Eigen::Vector3d &pa,
                        const Eigen::Vector3d &pb,
                        const Eigen::Matrix<double, 6, 4> &bd,
                        Eigen::MatrixX4d &hplanes,
                        bool use_optimized = false)
        {
            Eigen::Matrix3Xd star_convex_vertices;
            Eigen::Matrix3Xd convex_vertices;
            const Eigen::Vector3d query_point = 0.5 * (pa + pb);
            if (!buildConvexVertices(star_mesh, query_point, star_convex_vertices, convex_vertices))
            {
                hplanes.resize(0, 4);
                return;
            }

            const size_t ori_m = convex_vertices.cols() / 3;
            const size_t star_nv = star_convex_vertices.cols();
            
            // Preallocate storage
            hplanes.resize(ori_m + 6, 4);
            Eigen::VectorXd distances = Eigen::VectorXd::Constant(ori_m, use_optimized ? 100.0 : -1.0);
            
            // Extract triangle matrices
            Eigen::Matrix3Xd A(3, ori_m), B(3, ori_m), C(3, ori_m);
            extractTriangleMatrices(convex_vertices, A, B, C);
            
            // Compute normals (vectorized)
            Eigen::Matrix3Xd BC = B - C;
            Eigen::Matrix3Xd AC = A - C;
            Eigen::Matrix3Xd normals(3, ori_m);
            normals.row(0) = BC.row(1).cwiseProduct(AC.row(2)) - BC.row(2).cwiseProduct(AC.row(1));
            normals.row(1) = BC.row(2).cwiseProduct(AC.row(0)) - BC.row(0).cwiseProduct(AC.row(2));
            normals.row(2) = BC.row(0).cwiseProduct(AC.row(1)) - BC.row(1).cwiseProduct(AC.row(0));
            
            // Normalize and orient
            Eigen::RowVectorXd norms = normals.colwise().norm();
            for (size_t j = 0; j < ori_m; ++j) {
                if (norms(j) > 1e-12) {
                    normals.col(j) /= norms(j);
                    if (normals.col(j).dot(C.col(j)) > 0.0)
                        normals.col(j) = -normals.col(j);
                }
            }

            // Bounding box limits (for optimized version)
            double lx, ly, lz, ux, uy, uz;
            if (use_optimized) {
                lx = bd(1, 3); ly = bd(3, 3); lz = bd(5, 3);
                ux = -bd(0, 3); uy = -bd(2, 3); uz = -bd(4, 3);
            }

            // Main intersection loop
            for (size_t i = 0; i < star_nv; ++i) {
                const Eigen::Vector3d ap = star_convex_vertices.col(i);
                
                // Optional bounding box check
                if (use_optimized) {
                    const Eigen::Vector3d shifted = ap + query_point;
                    if (shifted(0) < lx || shifted(0) > ux ||
                        shifted(1) < ly || shifted(1) > uy ||
                        shifted(2) < lz || shifted(2) > uz)
                    {
                        continue;
                    }
                }
                
                if ((ap.array() == 0).any()) continue;
                
                for (size_t j = 0; j < ori_m; ++j) {
                    if (!isIntersectTriangle(ap, A.col(j), B.col(j), C.col(j)))
                        continue;

                    double d;
                    Eigen::Vector3d normal;
                    
                    if (use_optimized) {
                        d = ap.norm();
                        normal = -ap / d;
                        
                        if (d < distances[j]) {
                            distances[j] = d;
                            hplanes.row(j).head<3>() = -normal.transpose();
                            hplanes(j, 3) = -d + normal.dot(query_point);
                        }
                    } else {
                        d = (ap - C.col(j)).dot(normals.col(j));
                        
                        if (d > distances[j]) {
                            distances[j] = d;
                            hplanes.row(j).head<3>() = -normals.col(j).transpose();
                            hplanes(j, 3) = normals.col(j).dot(ap + query_point);
                        }
                    }
                }
            }

            // Fallback plane computation for triangles without valid intersections
            for (size_t n = 0; n < ori_m; ++n) {
                const bool needs_fallback = use_optimized ? (distances[n] >= 100.0) : (distances[n] <= 0.0);
                
                if (needs_fallback) {
                    hplanes.block<1, 3>(n, 0) = -normals.col(n).transpose();
                    hplanes(n, 3) = normals.col(n).dot(C.col(n) + query_point);
                }
            }

            // Add bounding box planes
            hplanes.block<6, 4>(ori_m, 0) = bd;
        }

        /**
         * @brief Check if point p lies inside the triangular prism defined by triangle ABC 
         *        extruded from the origin (tests if ray from origin through p intersects triangle)
         * Tetrahedron vertices
         */
        // isIntersectTriangle assumes all inputs are in the same local frame.
        bool isIntersectTriangle(const Eigen::Vector3d &p, const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c) const
        {
            // Compute determinant once
            const double d = a.x() * (b.y() * c.z() - c.y() * b.z()) 
                           - a.y() * (b.x() * c.z() - c.x() * b.z()) 
                           + a.z() * (b.x() * c.y() - c.x() * b.y());
            
            if (std::abs(d) < 1e-12) return false; // Use epsilon instead of exact zero
            
            const double inv_d = 1.0 / d; // Single division, reuse multiplications
            
            // Lambda 1: First barycentric coordinate
            const double m11 = (b.y() * c.z() - c.y() * b.z()) * inv_d;
            const double m12 = (c.x() * b.z() - b.x() * c.z()) * inv_d;
            const double m13 = (b.x() * c.y() - c.x() * b.y()) * inv_d;
            const double lambda1 = p.x() * m11 + p.y() * m12 + p.z() * m13;
            if (lambda1 < 0) return false;
            
            // Lambda 2: Second barycentric coordinate
            const double m21 = (c.y() * a.z() - a.y() * c.z()) * inv_d;
            const double m22 = (a.x() * c.z() - c.x() * a.z()) * inv_d;
            const double m23 = (a.y() * c.x() - a.x() * c.y()) * inv_d;
            const double lambda2 = p.x() * m21 + p.y() * m22 + p.z() * m23;
            if (lambda2 < 0) return false;
            
            // Lambda 3: Third barycentric coordinate (optimized check)
            // Since λ₁ + λ₂ + λ₃ = 1, we have λ₃ = 1 - λ₁ - λ₂
            // We need λ₃ > 0, so: 1 - λ₁ - λ₂ > 0, or λ₁ + λ₂ < 1
            if (lambda1 + lambda2 > 1.0) return false;
            
            return true;
        }

        // directly use ellipsoid to get polytopes from triangle obstacles
        // not normalized polytopes
        // triangles are in global frame; query_point is global.
        inline bool findPolytope(const TriangleVec &triangles,
                                 const Eigen::Vector3d &pa,
                                 const Eigen::Vector3d &pb,
                                 const Eigen::MatrixX4d &bd,
                                 Eigen::MatrixX4d &hPoly,
                                 const double epsilon = 1.0e-6)
        {
            if (triangles.empty() || bd.rows() == 0)
            {
                hPoly.resize(0, 4);
                return false;
            }
            Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
            Eigen::Vector3d d = 0.5 * (pa + pb);
            Eigen::Vector3d r = (pb - pa).cwiseAbs() * 0.5;

            // for r, only keep the longest axis unchanged, and other two set as 0.1
            double max_r = r.maxCoeff();
            for (int i = 0; i < 3; ++i)
            {
                if (r(i) < max_r)
                {
                    r(i) = 0.1;
                }
                else
                {
                    r(i) = 0.5 * max_r;
                }
            }
            std::cout << "[findPolytope] Ellipsoid center: " << d.transpose()
                      << ", radii: " << r.transpose() << std::endl;
            // const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
            // const Eigen::Vector3d d = Eigen::Vector3d::Zero();
            // const Eigen::Vector3d r = Eigen::Vector3d::Constant(0.1);

            // convert the space to ball space
            const Eigen::Matrix3d forward = r.cwiseInverse().asDiagonal() * R.transpose();

            const int N = triangles.size();
            Eigen::MatrixX4d forwardH(N, 4);
            int nH = 0;

            Eigen::MatrixX4d tangents(N, 4);
            Eigen::VectorXd distRs(N);

            // For each triangle, find the closest point on the triangle to the ellipsoid center
            // and compute the tangent plane
            for (int i = 0; i < N; i++)
            {
                const Triangle &tri = triangles[i];

                // Transform triangle vertices to ball space
                Eigen::Vector3d forwardA = forward * (tri._a - d);
                Eigen::Vector3d forwardB = forward * (tri._b - d);
                Eigen::Vector3d forwardC = forward * (tri._c - d);

                // Find closest point on triangle to origin in ball space
                Eigen::Vector3d closestPt = closestSeparatingPointToOrigin(forwardA, forwardB, forwardC);

                distRs(i) = closestPt.norm();
                if (distRs(i) <= epsilon)
                {
                    distRs(i) = std::numeric_limits<double>::infinity();
                    continue;
                }
                tangents(i, 3) = -distRs(i);
                tangents.block<1, 3>(i, 0) = closestPt.transpose() / distRs(i);
            }

            Eigen::Matrix<uint8_t, -1, 1> triFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(N, 1);

            nH = 0;

            bool completed = false;
            int triMinId = 0;
            double minSqrR = INFINITY;
            if (distRs.size() != 0)
            {
                minSqrR = distRs.minCoeff(&triMinId);
            }

            // std::cout << "[findPolytope] Initial minSqrR: " << minSqrR << ", triMinId: " << triMinId << std::endl;

            for (int i = 0; !completed && i < N; ++i)
            {
                if (!std::isfinite(minSqrR))
                {
                    break;
                }
                forwardH.row(nH) = tangents.row(triMinId);
                triFlags(triMinId) = 0;

                completed = true;
                minSqrR = INFINITY;

                for (int j = 0; j < N; ++j)
                {
                    if (triFlags(j))
                    {
                        const Triangle &tri = triangles[j];

                        // Transform triangle vertices to ball space
                        Eigen::Vector3d forwardA = forward * (tri._a - d);
                        Eigen::Vector3d forwardB = forward * (tri._b - d);
                        Eigen::Vector3d forwardC = forward * (tri._c - d);

                        // Check if all vertices satisfy the current hyperplane
                        double distA = forwardH.block<1, 3>(nH, 0).dot(forwardA) + forwardH(nH, 3);
                        double distB = forwardH.block<1, 3>(nH, 0).dot(forwardB) + forwardH(nH, 3);
                        double distC = forwardH.block<1, 3>(nH, 0).dot(forwardC) + forwardH(nH, 3);

                        // If entire triangle is satisfied, mark as inactive
                        if (distA > -epsilon && distB > -epsilon && distC > -epsilon)
                        {
                            triFlags(j) = 0;
                        }
                        else
                        {
                            completed = false;
                            if (minSqrR > distRs(j))
                            {
                                triMinId = j;
                                minSqrR = distRs(j);
                            }
                        }
                    }
                }
                ++nH;
            }

            if (nH <= 0)
            {
                hPoly = bd;
                return true;
            }

            hPoly.resize(nH + bd.rows(), 4);
            for (int i = 0; i < nH; ++i)
            {
                hPoly.block<1, 3>(i, 0) = forwardH.block<1, 3>(i, 0) * forward;
                hPoly(i, 3) = forwardH(i, 3) - hPoly.block<1, 3>(i, 0).dot(d);
            }
            hPoly.block(nH, 0, bd.rows(), 4) = bd;

            // Ensure endpoints are inside the polytope.
            for (Eigen::Index i = 0; i < hPoly.rows(); ++i)
            {
                const Eigen::Vector3d n = hPoly.row(i).head<3>().transpose();
                const double nn = n.norm();
                if (nn <= 1e-12)
                {
                    continue;
                }
                const double d0 = hPoly(i, 3);
                const double v0 = n.dot(pa) + d0;
                const double v1 = n.dot(pb) + d0;
                const double vmax = std::max(v0, v1);
                if (vmax > -epsilon)
                {
                    hPoly(i, 3) -= (vmax + epsilon);
                }
            }

            return true;
        }

        // Find closest point from a polytope (from QuickHull output) to the origin
        // vertices: 3xN matrix where every 3 consecutive columns form a triangle
        // vertices are in local frame (origin-centered).
        inline Eigen::Vector3d closestPointOnPolytopeToOrigin(
            const Eigen::Matrix3Xd &vertices)
        {
            Eigen::Vector3d closestPoint = Eigen::Vector3d::Zero();
            double minDistSq = std::numeric_limits<double>::max();

            int numTriangles = vertices.cols() / 3;

            for (int i = 0; i < numTriangles; ++i)
            {
                // Get the three vertices of this triangle
                const Eigen::Vector3d &a = vertices.col(i * 3);
                const Eigen::Vector3d &b = vertices.col(i * 3 + 1);
                const Eigen::Vector3d &c = vertices.col(i * 3 + 2);

                // Find closest point on this triangle
                Eigen::Vector3d point = closestSeparatingPointToOrigin(a, b, c);

                // Compute squared distance to origin
                double distSq = point.squaredNorm();

                // Update if this is closer
                if (distSq < minDistSq)
                {
                    closestPoint = point;
                    minDistSq = distSq;
                }
            }

            return closestPoint;
        }

        // a, b, c are in local frame (origin-centered).
        inline Eigen::Vector3d closestSeparatingPointToOrigin(
            const Eigen::Vector3d &a,
            const Eigen::Vector3d &b,
            const Eigen::Vector3d &c)
        {
            const Eigen::Vector3d ab = b - a;
            const Eigen::Vector3d ac = c - a;
            const Eigen::Vector3d ap = -a;

            double d1 = ab.dot(ap);
            double d2 = ac.dot(ap);
            if (d1 <= 0.0 && d2 <= 0.0)
                return a;

            const Eigen::Vector3d bp = -b;
            double d3 = ab.dot(bp);
            double d4 = ac.dot(bp);
            if (d3 >= 0.0 && d4 <= d3)
                return b;

            double vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
            {
                double v = d1 / (d1 - d3);
                return a + v * ab;
            }

            const Eigen::Vector3d cp = -c;
            double d5 = ab.dot(cp);
            double d6 = ac.dot(cp);
            if (d6 >= 0.0 && d5 <= d6)
                return c;

            double vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
            {
                double w = d2 / (d2 - d6);
                return a + w * ac;
            }

            double va = d3 * d6 - d5 * d4;
            if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
            {
                double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                return b + w * (c - b);
            }

            double denom = 1.0 / (va + vb + vc);
            double v = vb * denom;
            double w = vc * denom;
            return a + ab * v + ac * w;
        }



        // Alternative: Use Eigen's select/indexing if you want to avoid the loop
        // (May or may not be faster depending on data size and cache patterns)
        void filterH(const Eigen::MatrixX4d &hPoly,
                    Eigen::MatrixX4d &hPolyFiltered,
                    double epsilon = 1e-8) const
        {
            const Eigen::Index H = hPoly.rows();
            
            if (H < 4 || !hPoly.allFinite())
            {
                hPolyFiltered.resize(0, 4);
                return;
            }

            Eigen::Matrix3Xd vPoly;
            geo_utils::enumerateVs(hPoly, vPoly);
            
            if (vPoly.cols() == 0)
            {
                hPolyFiltered.resize(0, 4);
                return;
            }

            Eigen::MatrixXd values = (hPoly.leftCols<3>() * vPoly).colwise() + hPoly.col(3);
            
            std::vector<Eigen::Index> activeIndices;
            activeIndices.reserve(H); // Avoid reallocations
            
            for (Eigen::Index i = 0; i < H; ++i)
            {
                if ((values.row(i).array().abs() < epsilon).any())
                    activeIndices.push_back(i);
            }
            
            hPolyFiltered.resize(activeIndices.size(), 4);
            for (size_t i = 0; i < activeIndices.size(); ++i)
                hPolyFiltered.row(i) = hPoly.row(activeIndices[i]);
        }



        EIGEN_STRONG_INLINE
        // bd is global-frame AABB; samples are returned in global frame.
        void forEachBoundingFaceSampleRadial(const Eigen::Matrix<double, 6, 4> &bd,
                                            Eigen::Matrix3Xd &out,
                                            int n_samples = 1000,
                                            double scale = 1.0,
                                            double top_bottom_keep = 0.1)
        {
            // Bounds
            const double lx = bd(1, 3), ly = bd(3, 3), lz = bd(5, 3);
            const double ux = -bd(0, 3), uy = -bd(2, 3), uz = -bd(4, 3);

            // Center and half-extents
            const double cx = 0.5 * (lx + ux);
            const double cy = 0.5 * (ly + uy);
            const double cz = 0.5 * (lz + uz);
            const double half_dx = 0.5 * (ux - lx);
            const double half_dy = 0.5 * (uy - ly);
            const double half_dz = 0.5 * (uz - lz);

            const int corner_count = 8;
            const int total_samples = n_samples + corner_count;
            out.resize(3, total_samples);

            // Always include all 8 corners first
            out.col(0) = Eigen::Vector3d(lx, ly, lz);
            out.col(1) = Eigen::Vector3d(lx, ly, uz);
            out.col(2) = Eigen::Vector3d(lx, uy, lz);
            out.col(3) = Eigen::Vector3d(lx, uy, uz);
            out.col(4) = Eigen::Vector3d(ux, ly, lz);
            out.col(5) = Eigen::Vector3d(ux, ly, uz);
            out.col(6) = Eigen::Vector3d(ux, uy, lz);
            out.col(7) = Eigen::Vector3d(ux, uy, uz);

            if (n_samples <= 0)
            {
                out.conservativeResize(3, corner_count);
                return;
            }

            // Golden ratio and constants
            const double phi = 1.6180339887498948482;
            const double two_pi = 2.0 * M_PI;
            
            // Adjust sphere sample count to account for rejection
            const double expected_rejection_rate = 1.0 - (top_bottom_keep + (1.0 - top_bottom_keep) * 0.5);
            const int sphere_n = static_cast<int>(n_samples / (1.0 - expected_rejection_rate)) + 100;
            const double inv_sphere_n = 1.0 / double(sphere_n);

            std::mt19937 rng(12345);
            std::uniform_real_distribution<double> uni01(0.0, 1.0);

            int out_count = corner_count;
            
            // Single pass: generate sphere points and project to box faces
            for (int idx = 0; idx < sphere_n && out_count < total_samples; ++idx)
            {
                // Spherical Fibonacci point on unit sphere
                const double y = 1.0 - 2.0 * (double(idx) + 0.5) * inv_sphere_n;
                const double r = std::sqrt(1.0 - y * y);
                const double theta = two_pi * double(idx) / phi;

                const double dx = r * std::cos(theta);
                const double dz = r * std::sin(theta);
                const double dy = y;

                // Find which face this ray hits first (closest positive t)
                const double tx_pos = (dx > 1e-10) ? half_dx / dx : 1e10;
                const double tx_neg = (dx < -1e-10) ? -half_dx / dx : 1e10;
                const double ty_pos = (dy > 1e-10) ? half_dy / dy : 1e10;
                const double ty_neg = (dy < -1e-10) ? -half_dy / dy : 1e10;
                const double tz_pos = (dz > 1e-10) ? half_dz / dz : 1e10;
                const double tz_neg = (dz < -1e-10) ? -half_dz / dz : 1e10;

                double t = tx_pos;
                if (tx_neg < t) t = tx_neg;
                if (ty_pos < t) t = ty_pos;
                if (ty_neg < t) t = ty_neg;
                if (tz_pos < t) t = tz_pos;
                if (tz_neg < t) t = tz_neg;

                // Reject top/bottom samples probabilistically
                const bool on_top_or_bottom = (t == tz_pos || t == tz_neg);
                if (on_top_or_bottom && uni01(rng) > top_bottom_keep)
                {
                    continue;
                }

                // Project to surface and store
                const double ts = t * scale;
                out.col(out_count++) = Eigen::Vector3d(cx + ts * dx,
                                                    cy + ts * dy,
                                                    cz + ts * dz);
            }

            // If we still need more samples (unlikely), truncate
            if (out_count < total_samples)
            {
                out.conservativeResize(3, out_count);
            }
        }

        // Append the 8 AABB corner points from bd to an existing 3xN sample matrix.
        static void appendBdCornerSamples(const Eigen::Matrix<double, 6, 4> &bd,
                                          Eigen::Matrix3Xd &samples)
        {
            if (samples.rows() == 0)
            {
                samples.resize(3, 0);
            }
            if (samples.rows() != 3)
            {
                std::cerr << "[appendBdCornerSamples] Expected 3xN samples." << std::endl;
                return;
            }

            const double lx = bd(1, 3), ly = bd(3, 3), lz = bd(5, 3);
            const double ux = -bd(0, 3), uy = -bd(2, 3), uz = -bd(4, 3);

            Eigen::Matrix<double, 3, 8> corners;
            corners.col(0) = Eigen::Vector3d(lx, ly, lz);
            corners.col(1) = Eigen::Vector3d(lx, ly, uz);
            corners.col(2) = Eigen::Vector3d(lx, uy, lz);
            corners.col(3) = Eigen::Vector3d(lx, uy, uz);
            corners.col(4) = Eigen::Vector3d(ux, ly, lz);
            corners.col(5) = Eigen::Vector3d(ux, ly, uz);
            corners.col(6) = Eigen::Vector3d(ux, uy, lz);
            corners.col(7) = Eigen::Vector3d(ux, uy, uz);

            const Eigen::Index prev_cols = samples.cols();
            samples.conservativeResize(3, prev_cols + corners.cols());
            samples.rightCols(corners.cols()) = corners;
        }

    };

}

#endif // GALAXY_HPP
