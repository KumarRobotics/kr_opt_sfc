#ifndef ROBUST_STAR_HPP
#define ROBUST_STAR_HPP

#include "star/star.hpp"
#include "star/star_vis.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>
#include <memory>

namespace Star
{

class RobustStarManager
{
public:
    RobustStarManager() = default;

    void init(float radius, float precision,
              const Eigen::Vector3d &lowCorner,
              const Eigen::Vector3d &highCorner,
              const float range,
              const float hz,
              const float safe_margin = 0.1f)
    {
        lowCorner_ = lowCorner;
        highCorner_ = highCorner;
        range_ = range;
        hz_ = hz;
        if (hz_ > 0.0 && std::isfinite(hz_))
        {
            const float dt = 1.0f / hz_;
            belief_decay_ = std::exp(-LAMBDA * dt);
        }
        else
        {
            belief_decay_ = 1.0f;
        }
        star_gen_.init(radius, precision, safe_margin);
        merge_distance_ = 0.5f * range_;
    }

    bool buildBoundedPointCloud(const Eigen::Vector3d &query_point,
                                const std::vector<Eigen::Vector3d> &points,
                                Eigen::Matrix<double, 6, 4> &bd,
                                Eigen::Matrix3Xd &pc) const
    {
        const Eigen::Vector3d lowerBound =
            (query_point.array() - range_).max(lowCorner_.array());
        const Eigen::Vector3d upperBound =
            (query_point.array() + range_).min(highCorner_.array());

        // Build bounds matrix
        bd.setZero();
        bd.col(0) << 1.0, -1.0, 0.0, 0.0, 0.0, 0.0;
        bd.col(1) << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0;
        bd.col(2) << 0.0, 0.0, 0.0, 0.0, 1.0, -1.0;
        bd.col(3) << -upperBound(0), lowerBound(0), 
                     -upperBound(1), lowerBound(1), 
                     -upperBound(2),  lowerBound(2);

        if (points.empty())
        {
            return false;
        }

        // Filter points within bounds
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> valid_pc;
        valid_pc.reserve(points.size());

        for (const Eigen::Vector3d &p : points)
        {
            if ((p.array() >= lowerBound.array()).all() &&
                (p.array() <= upperBound.array()).all())
            {
                valid_pc.push_back(p);
            }
        }

        if (valid_pc.empty())
        {
            return false;
        }

        // Convert to Eigen matrix
        pc.resize(3, static_cast<Eigen::Index>(valid_pc.size()));
        for (size_t i = 0; i < valid_pc.size(); ++i)
        {
            pc.col(static_cast<Eigen::Index>(i)) = valid_pc[i];
        }

        return true;
    }


    /*** Return values:
     * 0: no collision, no update needed
     * 1: refinement applied
     * 2: rebuild occurred
     */
    int update(const Eigen::Vector3d &query_point,
            const std::vector<Eigen::Vector3d> &points)
    {
        auto t_start = std::chrono::high_resolution_clock::now();
        
        Eigen::Matrix<double, 6, 4> bd;
        Eigen::Matrix3Xd pc;

        const bool has_pc = buildBoundedPointCloud(query_point, points, bd, pc);
        if (!has_pc)
        {
            pc.resize(3, 0);
        }

        int result = 0;

        // 1. Initialize current node on first call.
        if (!cur_node_valid_)
        {
            std::cout << "[RobustStarManager] Building initial star mesh at query point: "
                    << query_point.transpose() << std::endl;

            star_gen_.createNbd(query_point, pc, bd, cur_node_);
            cur_node_valid_ = true;

            // Seed the roadmap with the first node.
            star_nodes_.push_back(cur_node_);
            adjacency_.emplace_back();
            connectNewNode(static_cast<int>(star_nodes_.size() - 1));
            result = 2;
        }
        // 2. Rebuild current node if query point moved.
        else if ((query_point - cur_node_.center).norm() > 0.1)
        {
            std::cout << "[RobustStarManager] Query point changed, rebuilding current star mesh."
                    << std::endl;
            rebuildAndRegisterNode(query_point, pc, bd);
            result = 2;
        }
        // 3. Check if inside current kernel.
        else if (sfc_utils::checkInsidePoly(query_point, cur_node_.star_kernel))
        {
            // Inside kernel: refine current node if new points available.
            if (has_pc)
            {
                refineStarNode(cur_node_, query_point, pc, bd);
                result = 1;
            }
            else
            {
                result = 0;
            }
        }
        // 4. Outside kernel: rebuild current node.
        else
        {
            std::cout << "[RobustStarManager] Query point outside current star kernel, creating new star mesh."
                      << std::endl;
            rebuildAndRegisterNode(query_point, pc, bd);
            result = 2;
        }

        if constexpr (DEBUG_TIMING)
        {
            const auto t_end = std::chrono::high_resolution_clock::now();
            const double t_total = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            update_total_ms_ += t_total;
            ++update_count_;
            const double avg_ms = update_total_ms_ / static_cast<double>(update_count_);
            std::cout << "[Timing] Total: " << t_total
                      << " ms, Avg: " << avg_ms
                      << " ms (" << update_count_ << " updates)"
                      << std::endl;
        }

        return result;
    }

    const std::vector<StarNode, Eigen::aligned_allocator<StarNode>> &getStarNodes() const
    {
        return star_nodes_;
    }

    void clearStarNodes()
    {
        star_nodes_.clear();
        adjacency_.clear();
        cur_node_valid_ = false;
        update_total_ms_ = 0.0;
        update_count_ = 0;
    }

    const std::vector<int> &getNeighbors(size_t idx) const
    {
        return adjacency_.at(idx);
    }

    const StarNode *getCurrentStarNode() const
    {
        if (!cur_node_valid_)
        {
            return nullptr;
        }
        return &cur_node_;
    }

    bool buildRoutePolytopes(const std::vector<Eigen::Vector3d> &route,
                             int method,
                             std::vector<Eigen::MatrixX4d> &hpolys)
    {
        hpolys.clear();
        if (!cur_node_valid_ || cur_node_.star_mesh.empty() || route.size() < 2)
        {
            std::cout << "[RobustStarManager] buildRoutePolytopes failed: invalid current star node or route."
                      << std::endl;
            return false;
        }

        if (method < 0 || method > 2)
        {
            method = 0;
        }

        hpolys.reserve(route.size());
        for (size_t i = 0; i + 1 < route.size(); ++i)
        {
            Eigen::MatrixX4d hpoly;
            std::cout << "[RobustStarManager] Finding inner polytope for route point: "
                      << route[i].transpose() << std::endl;
            if (!star_gen_.findInnerPoly(cur_node_, route[i], route[i+1], hpoly, method))
            {
                return false;
            }
            hpolys.emplace_back(hpoly);
        }
        return !hpolys.empty();
    }

    void initVisualizer(ros::NodeHandle &nh, int id = 0)
    {
        visualizer_.reset(new star_vis(nh, id));
    }

    void visualizeCurrent(const Eigen::Vector3d &seed_pt, bool clear_all = true)
    {
        if (!visualizer_)
        {
            return;
        }
        if (clear_all)
        {
            visualizer_->clearAll();
        }
        visualizer_->visualizeStartGoal(seed_pt, 0.3, 1);

        const StarNode *current = getCurrentStarNode();
        if (current)
        {
            visualizer_->visualizeStarPolytope({current->star_mesh});
            visualizer_->visualizePolytope({current->star_kernel});
            visualizer_->visualizeBoundingBox(current->bd);
            visualizer_->visualizeStarKernel({current->star_kernel});
        }
        const Eigen::Matrix3Xd centers = getStarCentersMatrix();
        if (centers.cols() > 0)
        {
            visualizer_->visualizePoints(centers);
        }
    }

    Eigen::Matrix3Xd getStarCentersMatrix() const
    {
        const Eigen::Index n = static_cast<Eigen::Index>(star_nodes_.size());
        Eigen::Matrix3Xd centers(3, n);
        for (Eigen::Index i = 0; i < n; ++i)
        {
            centers.col(i) = star_nodes_[i].center;
        }
        return centers;
    }

private:
    Eigen::Vector3d lowCorner_, highCorner_;
    float range_ = 3.0f;
    float hz_ = 1.0f;
    float belief_decay_ = 1.0f;
    float merge_distance_ = 0.0f;
    Star::Corridor star_gen_;
    std::vector<StarNode, Eigen::aligned_allocator<StarNode>> star_nodes_;
    std::vector<std::vector<int>> adjacency_;
    StarNode cur_node_;
    bool cur_node_valid_ = false;
    std::unique_ptr<star_vis> visualizer_;
    double update_total_ms_ = 0.0;
    size_t update_count_ = 0;

    // Configuration constants
    static constexpr double BETA = 1e-3;          // belief penalty
    static constexpr double LAMBDA = 0.05;        // belief decay
    static constexpr double L_RESET = -1.0;       // belief threshold for rebuild
    static constexpr double BELIEF_MIN = -10.0;   // belief clamp min
    static constexpr double BELIEF_MAX = 10.0;    // belief clamp max
    static constexpr double SUPPORT_THRESHOLD = 1.0; // distance threshold for support points
    static constexpr bool DEBUG_REFINE_POINTS = false;
    // Per-stage timing/diagnostic prints (log_step, "[Timing] ..." lines) add
    // real per-call overhead (each std::endl flushes) since update() runs
    // continuously at RunHz; keep them off unless actively profiling.
    static constexpr bool DEBUG_TIMING = false;

    void refineStarNode(StarNode &star_node,
                        const Eigen::Vector3d &query_point,
                        const Eigen::Matrix3Xd &pc,
                        const Eigen::Matrix<double, 6, 4> &bd)
    {
        auto t_step = std::chrono::high_resolution_clock::now();
        const auto log_step = [&](const char *label)
        {
            if constexpr (DEBUG_TIMING)
            {
                const auto t_now = std::chrono::high_resolution_clock::now();
                const double t_ms =
                    std::chrono::duration<double, std::milli>(t_now - t_step).count();
                std::cout << "[Timing] refineStarNode " << label << ": " << t_ms << " ms"
                          << std::endl;
                t_step = t_now;
            }
        };
        const Eigen::Index N = pc.cols();
        const int M = static_cast<int>(star_node.star_mesh.size());
        if (M == 0 || N == 0)
        {
            log_step("early_exit_empty_input");
            return;
        }

        // Initialize belief vector if needed
        if (star_node.belief.size() != M)
        {
            star_node.belief = Eigen::VectorXd::Zero(M);
        }
        log_step("init_belief");

        // Classify points and update beliefs
        std::vector<int> pc_support;
        std::vector<int> pc_inside;
        size_t inside_count = 0;
        size_t skipped_reverse_poly = 0;
        pc_support.reserve(N);
        pc_inside.reserve(N);

        Eigen::VectorXd new_belief = belief_decay_ * star_node.belief;

        constexpr double tetra_eps = 1e-9;
        Eigen::Matrix3Xd cross_bc(3, M);
        Eigen::Matrix3Xd cross_ca(3, M);
        Eigen::Matrix3Xd cross_ab(3, M);
        Eigen::VectorXd v0(M);
        for (int i = 0; i < M; ++i)
        {
            const Triangle &tri = star_node.star_mesh[i];
            const Eigen::Vector3d ad = tri._a - query_point;
            const Eigen::Vector3d bd = tri._b - query_point;
            const Eigen::Vector3d cd = tri._c - query_point;
            cross_bc.col(i) = bd.cross(cd);
            cross_ca.col(i) = cd.cross(ad);
            cross_ab.col(i) = ad.cross(bd);
            v0(i) = ad.dot(cross_bc.col(i));
        }
        log_step("precompute_geometry");

        Eigen::MatrixX3d kernel_A;
        Eigen::VectorXd kernel_b;
        Eigen::VectorXd kernel_violation;
        const bool has_kernel = star_node.star_kernel.rows() > 0;
        if (has_kernel)
        {
            const Eigen::ArrayXd hNorm = star_node.star_kernel.leftCols<3>().rowwise().norm();
            kernel_A = star_node.star_kernel.leftCols<3>().array().colwise() / hNorm;
            kernel_b = star_node.star_kernel.rightCols<1>().array() / hNorm;
            kernel_violation.resize(kernel_A.rows());
        }


        const Eigen::MatrixX4d &reverse_poly = star_node.reverse_convex_poly;
        Eigen::MatrixX3d reverse_A;
        Eigen::VectorXd reverse_b;
        if (reverse_poly.rows() > 0)
        {
            const Eigen::ArrayXd hNorm = reverse_poly.leftCols<3>().rowwise().norm();
            reverse_A = reverse_poly.leftCols<3>().array().colwise() / hNorm;
            reverse_b = -reverse_poly.rightCols<1>().array() / hNorm;
        }
        log_step("kernel_prep");

        double max_radius_sq = 0.0;
        for (Eigen::Index i = 0; i < star_node.unique_sconvex_vertices.cols(); ++i)
        {
            const double r_sq = star_node.unique_sconvex_vertices.col(i).squaredNorm();
            if (r_sq > max_radius_sq)
            {
                max_radius_sq = r_sq;
            }
        }
        const double support_radius_sq =
            max_radius_sq + SUPPORT_THRESHOLD * SUPPORT_THRESHOLD;


        for (Eigen::Index k = 0; k < N; ++k)
        {
            const Eigen::Vector3d pc_x = pc.col(k);
            const Eigen::Vector3d pd = pc_x - query_point;
            if (pd.squaredNorm() > support_radius_sq)
            {
                continue;
            }
            
            // Check tetrahedra containment
            double dist_in_tetra = 0.0;
            bool has_dist_in_tetra = false;
            bool inside = false;

            // reverse_convex_poly was built by flipping points around star_node.center
            // (the query point at the node's last rebuild); flip around that same
            // center here too, not the live query_point, which can have drifted from
            // it by up to the 0.1 rebuild threshold.
            Eigen::Vector3d pc_x_flip = pc_x;
            star_gen_.flipMap(pc_x_flip, star_node.center);
            bool inside_reverse_poly = true;
            for (Eigen::Index j = 0; j < reverse_A.rows(); ++j)
            {
                if (reverse_A.row(j).dot(pc_x_flip) - reverse_b(j) > 1.0e-6)
                {
                    inside_reverse_poly = false;
                    break;
                }
            }
            if (inside_reverse_poly)
            {
                ++skipped_reverse_poly;
                continue;
            }

            for (int i = 0; i < M; ++i)
            {
                const double v1 = pd.dot(cross_bc.col(i));
                const double v2 = pd.dot(cross_ca.col(i));
                const double v3 = pd.dot(cross_ab.col(i));
                const double v4 = v0(i) - v1 - v2 - v3;

                const bool same_sign =
                    (v0(i) >= -tetra_eps && v1 >= -tetra_eps && v2 >= -tetra_eps &&
                     v3 >= -tetra_eps && v4 >= -tetra_eps) ||
                    (v0(i) <= tetra_eps && v1 <= tetra_eps && v2 <= tetra_eps &&
                     v3 <= tetra_eps && v4 <= tetra_eps);

                if (!same_sign)
                {
                    continue;
                }

                const double sum = std::abs(v1) + std::abs(v2) + std::abs(v3) + std::abs(v4);
                const double abs_v0 = std::abs(v0(i));
                if (std::abs(sum - abs_v0) > tetra_eps * std::max(1.0, abs_v0))
                {
                    continue;
                }

                // A point can only be inside one tetrahedron of the star fan (barring
                // measure-zero shared-face boundary cases), so stop scanning the
                // remaining triangles once one match is found.
                inside = true;
                new_belief(i) -= BETA;
                if (DEBUG_REFINE_POINTS)
                {
                    const Triangle &tri = star_node.star_mesh[i];
                    dist_in_tetra = std::abs(tri._normal.dot(pc_x - tri._a));
                    has_dist_in_tetra = true;
                }
                break;
            }

            if (inside)
            {
                ++inside_count;
                pc_inside.push_back(static_cast<int>(k));
                if (DEBUG_REFINE_POINTS)
                {
                    std::cout << "[RefineStarNode] Point " << k
                              << ", inside: " << inside;
                    if (has_dist_in_tetra)
                    {
                        std::cout << ", dist_in_tetra: " << dist_in_tetra;
                    }
                    std::cout << std::endl;
                }
            }
            else if (has_kernel)
            {
                kernel_violation.noalias() = kernel_A * pc_x;
                kernel_violation += kernel_b;
                const double max_violation = kernel_violation.maxCoeff();
                if (max_violation <= SUPPORT_THRESHOLD)
                {
                    // Point is outside but close to the mesh (support region)
                    pc_support.push_back(k);
                }
            }
        }
        log_step("classify_points");
        if constexpr (DEBUG_TIMING)
        {
            std::cout << "[RefineStarNode] Skipped by reverse convex poly: "
                      << skipped_reverse_poly << " / " << N << std::endl;
        }

        star_node.belief = new_belief.cwiseMax(BELIEF_MIN).cwiseMin(BELIEF_MAX);

        if (!pc_inside.empty())
        {
            if constexpr (DEBUG_TIMING)
            {
                std::cout << "[RefineStarNode] Refining star mesh with inside points: "
                          << pc_inside.size() << std::endl;
            }
            updateStarNodeWithPoints(pc, pc_inside, star_node);
            log_step("shrink_inside");
            return;
        }

        // Rebuild if belief is too low
        if (star_node.belief.minCoeff() < L_RESET)
        {
            std::cout << "[RefineStarNode] Rebuilding star mesh due to low belief." << std::endl;
            star_gen_.createNbd(query_point, pc, bd, star_node);
            log_step("rebuild_low_belief");
            return;
        }

        // Only use support points to update the star mesh
        if (!pc_support.empty())
        {
            if constexpr (DEBUG_TIMING)
            {
                std::cout << "[RefineStarNode] Updating star mesh with " << pc_support.size()
                          << " support points." << std::endl;
            }
            updateStarNodeWithPoints(pc, pc_support, star_node);
        }
        log_step("support_update");

        if constexpr (DEBUG_TIMING)
        {
            std::cout << "[RefineStarNode] Updated beliefs min and max is: "
                      << star_node.belief.minCoeff() << ", "
                      << star_node.belief.maxCoeff() << std::endl;
        }
        log_step("finalize");
    }


    void updateStarNodeWithPoints(const Eigen::Matrix3Xd &pc,
                                  const std::vector<int> &idx,
                                  StarNode &star_node)
    {
        if (idx.empty()) return;

        Eigen::Matrix3Xd pts(3, idx.size());
        for (size_t i = 0; i < idx.size(); ++i)
        {
            pts.col(i) = pc.col(idx[i]);
        }

        star_gen_.updateStarNode(pts, star_node);
    }

    static void boundsFromBd(const Eigen::Matrix<double, 6, 4> &bd,
                             Eigen::Vector3d &min_bound,
                             Eigen::Vector3d &max_bound)
    {
        min_bound = Eigen::Vector3d(bd(1, 3), bd(3, 3), bd(5, 3));
        max_bound = Eigen::Vector3d(-bd(0, 3), -bd(2, 3), -bd(4, 3));
    }

    static bool boundsOverlap(const Eigen::Vector3d &min_a,
                              const Eigen::Vector3d &max_a,
                              const Eigen::Vector3d &min_b,
                              const Eigen::Vector3d &max_b)
    {
        return (max_a.array() >= min_b.array()).all() &&
               (max_b.array() >= min_a.array()).all();
    }

    static bool pointInsideBd(const Eigen::Matrix<double, 6, 4> &bd,
                              const Eigen::Vector3d &pt)
    {
        Eigen::Vector3d min_bound;
        Eigen::Vector3d max_bound;
        boundsFromBd(bd, min_bound, max_bound);
        return (pt.array() >= min_bound.array()).all() &&
               (pt.array() <= max_bound.array()).all();
    }

    int findNearbyNodeIndex(const StarNode &node) const
    {
        if (merge_distance_ <= 0.0 || star_nodes_.empty())
        {
            return -1;
        }

        int best_idx = -1;
        double best_dist = merge_distance_;
        Eigen::Vector3d min_new, max_new;
        boundsFromBd(node.bd, min_new, max_new);

        for (int i = 0; i < static_cast<int>(star_nodes_.size()); ++i)
        {
            Eigen::Vector3d min_old, max_old;
            boundsFromBd(star_nodes_[i].bd, min_old, max_old);
            if (!boundsOverlap(min_new, max_new, min_old, max_old))
            {
                continue;
            }

            const double dist = (star_nodes_[i].center - node.center).norm();
            if (dist <= best_dist)
            {
                best_dist = dist;
                best_idx = i;
            }
        }

        return best_idx;
    }

    void reconnectNode(int idx)
    {
        if (idx < 0 || idx >= static_cast<int>(star_nodes_.size()))
        {
            return;
        }

        adjacency_[idx].clear();

        Eigen::Vector3d min_new, max_new;
        boundsFromBd(star_nodes_[idx].bd, min_new, max_new);

        for (int i = 0; i < static_cast<int>(star_nodes_.size()); ++i)
        {
            if (i == idx)
            {
                continue;
            }

            auto &adj = adjacency_[i];
            adj.erase(std::remove(adj.begin(), adj.end(), idx), adj.end());

            Eigen::Vector3d min_old, max_old;
            boundsFromBd(star_nodes_[i].bd, min_old, max_old);
            if (!boundsOverlap(min_new, max_new, min_old, max_old))
            {
                continue;
            }

            adjacency_[idx].push_back(i);
            adjacency_[i].push_back(idx);
        }
    }


    void rebuildAndRegisterNode(const Eigen::Vector3d &query_point,
                                 const Eigen::Matrix3Xd &pc,
                                 const Eigen::Matrix<double, 6, 4> &bd)
    {
        star_gen_.createNbd(query_point, pc, bd, cur_node_);

        const bool has_roadmap = !star_nodes_.empty();
        const bool outside_last_bd =
            has_roadmap ? !pointInsideBd(star_nodes_.back().bd, query_point) : true;

        if (!outside_last_bd)
        {
            return;
        }

        std::cout << "\033[33m"
                << "[RobustStarManager] Adding new star mesh at query point: "
                << query_point.transpose();
        if (has_roadmap)
        {
            std::cout << " Last node at: " << star_nodes_.back().center.transpose();
        }
        std::cout << "\033[0m" << std::endl;

        if (!mergeOrReplaceRoadmapNode(cur_node_))
        {
            star_nodes_.push_back(cur_node_);
            adjacency_.emplace_back();
            connectNewNode(static_cast<int>(star_nodes_.size() - 1));
        }
    }

    bool mergeOrReplaceRoadmapNode(const StarNode &node)
    {
        const int idx = findNearbyNodeIndex(node);
        if (idx < 0)
        {
            return false;
        }

        std::cout << "\033[33m"
                  << "[RobustStarManager] Replacing nearby roadmap node at: "
                  << star_nodes_[idx].center.transpose()
                  << "\033[0m" << std::endl;

        star_nodes_[idx] = node;
        reconnectNode(idx);
        return true;
    }

    void connectNewNode(int new_idx)
    {
        if (new_idx <= 0) return;

        Eigen::Vector3d min_new, max_new;
        boundsFromBd(star_nodes_[new_idx].bd, min_new, max_new);

        for (int i = 0; i < new_idx; ++i)
        {
            Eigen::Vector3d min_old, max_old;
            boundsFromBd(star_nodes_[i].bd, min_old, max_old);
            if (!boundsOverlap(min_new, max_new, min_old, max_old))
            {
                continue;
            }
            adjacency_[new_idx].push_back(i);
            adjacency_[i].push_back(new_idx);
        }
    }

};

} // namespace Star

#endif // ROBUST_STAR_HPP
