#ifndef STAR_VIS_HPP
#define STAR_VIS_HPP

#include "sfc/sfc_vis.hpp"
#include "star/star.hpp"

class star_vis
{
private:
    sfc_vis base_;

    ros::NodeHandle nh_;
    std::string frame_id_;
    Eigen::Vector3d color_;

    ros::Publisher starKernelMeshPub;
    ros::Publisher starKernelEdgePub;
    ros::Publisher starPub;
    ros::Publisher bboxPub;
    ros::Publisher pointsPub;

public:
    star_vis() {}

    star_vis(ros::NodeHandle &nh_in, int id = 0)
        : base_(nh_in, id), nh_(nh_in)
    {
        std::string visualizer_topic = "/visualizer" + std::to_string(id);
        color_(0) = std::max((id % 2) / 2.0, 0.2);
        color_(1) = std::min((id % 5) / 5.0, 0.9);
        color_(2) = (id % 3) / 3.0;

        nh_.param("frame_id", frame_id_, std::string("map"));

        starKernelMeshPub = nh_.advertise<visualization_msgs::Marker>(visualizer_topic + "/star_kernel_mesh", 1000);
        starKernelEdgePub = nh_.advertise<visualization_msgs::Marker>(visualizer_topic + "/star_kernel_edge", 1000);
        starPub = nh_.advertise<visualization_msgs::MarkerArray>(visualizer_topic + "/star", 1000);
        bboxPub = nh_.advertise<visualization_msgs::Marker>(visualizer_topic + "/bounding_box", 10);
        pointsPub = nh_.advertise<visualization_msgs::Marker>(visualizer_topic + "/points", 10);
    }

    // delegate the markers RobustStarManager also needs from the base sfc_vis
    inline void clearAll() { base_.clearAll(); }

    inline void visualizeStartGoal(const Eigen::Vector3d &center,
                                   const double &radius,
                                   const int sg,
                                   Eigen::Vector3d color = Eigen::Vector3d(0.0, 0.3, 0.5))
    {
        base_.visualizeStartGoal(center, radius, sg, color);
    }

    inline void visualizePolytope(const std::vector<Eigen::MatrixX4d> &hPolys)
    {
        base_.visualizePolytope(hPolys);
    }

    inline void visualizeEllipsoids(const std::vector<Eigen::Matrix3d> &Rs,
                                    const std::vector<Eigen::Vector3d> &ds,
                                    const std::vector<Eigen::Vector3d> &rs)
    {
        base_.visualizeEllipsoids(Rs, ds, rs);
    }

    inline void visualizePoints(const Eigen::Matrix3Xd &pts)
    {
        visualization_msgs::Marker pointsMarker;

        pointsMarker.header.stamp = ros::Time::now();
        pointsMarker.header.frame_id = frame_id_;
        pointsMarker.pose.orientation.w = 1.00;
        pointsMarker.action = visualization_msgs::Marker::ADD;
        pointsMarker.id = 0;
        pointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        pointsMarker.ns = "points";
        pointsMarker.color.r = 0.0;
        pointsMarker.color.g = 0.0;
        pointsMarker.color.b = 0.8; //dark blue 
        pointsMarker.color.a = 0.8;
        pointsMarker.scale.x = 0.5;
        pointsMarker.scale.y = 0.5;
        pointsMarker.scale.z = 0.5;

        for (int i = 0; i < pts.cols(); i++)
        {
            geometry_msgs::Point point;
            point.x = pts.col(i)(0);
            point.y = pts.col(i)(1);
            point.z = pts.col(i)(2);
            pointsMarker.points.push_back(point);
        }

        pointsPub.publish(pointsMarker);
    }

    // bd is 6x4 matrix, six planes, each plane is represented as ax+by+cz+d=0
    inline void visualizeBoundingBox(const Eigen::Matrix<double, 6, 4> &bd)
    {
        visualization_msgs::Marker boxMarker;

        boxMarker.id = 0;
        boxMarker.type = visualization_msgs::Marker::LINE_LIST;
        boxMarker.header.stamp = ros::Time::now();
        boxMarker.header.frame_id = frame_id_;
        boxMarker.pose.orientation.w = 1.00;
        boxMarker.action = visualization_msgs::Marker::ADD;
        boxMarker.ns = "bounding_box";
        boxMarker.color.r = 0.0;
        boxMarker.color.g = 0.0;
        boxMarker.color.b = 0.0;
        boxMarker.color.a = 1.0;
        boxMarker.scale.x = 0.05;
        boxMarker.scale.y = 0.05;
        boxMarker.scale.z = 0.02;

        // Extract plane equations (assuming order: x_min, x_max, y_min, y_max, z_min, z_max)
        // Each plane: ax + by + cz + d = 0
        auto solve_intersection = [&](int p1, int p2, int p3) -> Eigen::Vector3d
        {
            Eigen::Matrix3d A;
            Eigen::Vector3d b;
            A.row(0) = bd.block<1, 3>(p1, 0);
            A.row(1) = bd.block<1, 3>(p2, 0);
            A.row(2) = bd.block<1, 3>(p3, 0);
            b << -bd(p1, 3), -bd(p2, 3), -bd(p3, 3);
            return A.colPivHouseholderQr().solve(b);
        };

        std::vector<Eigen::Vector3d> corners(8);
        corners[0] = solve_intersection(0, 2, 4);
        corners[1] = solve_intersection(1, 2, 4);
        corners[2] = solve_intersection(1, 3, 4);
        corners[3] = solve_intersection(0, 3, 4);
        corners[4] = solve_intersection(0, 2, 5);
        corners[5] = solve_intersection(1, 2, 5);
        corners[6] = solve_intersection(1, 3, 5);
        corners[7] = solve_intersection(0, 3, 5);

        const int edges[12][2] = {
            {0, 1}, {1, 2}, {2, 3}, {3, 0}, // bottom face
            {4, 5}, {5, 6}, {6, 7}, {7, 4}, // top face
            {0, 4}, {1, 5}, {2, 6}, {3, 7}  // vertical edges
        };

        for (const auto &e : edges)
        {
            geometry_msgs::Point p1, p2;
            p1.x = corners[e[0]](0);
            p1.y = corners[e[0]](1);
            p1.z = corners[e[0]](2);
            p2.x = corners[e[1]](0);
            p2.y = corners[e[1]](1);
            p2.z = corners[e[1]](2);
            boxMarker.points.push_back(p1);
            boxMarker.points.push_back(p2);
        }

        bboxPub.publish(boxMarker);
    }

    // visualize the star kernel (a convex H-representation polytope) on its
    // own dedicated topics so it doesn't collide with the base sfc mesh
    inline void visualizeStarKernel(const std::vector<Eigen::MatrixX4d> &hPolys)
    {
        Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0);
        for (const auto &hp : hPolys)
        {
            const Eigen::Matrix3Xd oldTris = mesh;
            sfc_utils::getMesh(hp, curTris);
            mesh.resize(3, oldTris.cols() + curTris.cols());
            mesh.leftCols(oldTris.cols()) = oldTris;
            mesh.rightCols(curTris.cols()) = curTris;
        }
        visualizeMeshOn(mesh, starKernelMeshPub, starKernelEdgePub);
    }

    // visualize the (possibly non-convex) star polytope mesh produced by RobustStarManager
    inline void visualizeStarPolytope(const std::vector<Star::TriangleVec> &star_meshes)
    {
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker clear_marker;
        clear_marker.header.stamp = ros::Time::now();
        clear_marker.header.frame_id = frame_id_;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        for (std::size_t idx = 0; idx < star_meshes.size(); ++idx)
        {
            const Star::TriangleVec &triangles = star_meshes[idx];
            if (triangles.empty())
            {
                continue;
            }

            const int tris = static_cast<int>(triangles.size());
            visualization_msgs::Marker starMarker, edgeMarker;

            starMarker.id = static_cast<int>(idx * 2);
            starMarker.header.stamp = ros::Time::now();
            starMarker.header.frame_id = frame_id_;
            starMarker.pose.orientation.w = 1.00;
            starMarker.action = visualization_msgs::Marker::ADD;
            starMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            starMarker.ns = "star_polytope";
            starMarker.color.r = color_(0) + 0.1;
            starMarker.color.g = color_(1) + 0.1;
            starMarker.color.b = color_(2) + 0.1;
            starMarker.color.a = 0.2;
            starMarker.scale.x = 1.0;
            starMarker.scale.y = 1.0;
            starMarker.scale.z = 1.0;
            starMarker.points.reserve(static_cast<size_t>(tris) * 3);

            geometry_msgs::Point point;
            for (const auto &tri : triangles)
            {
                point.x = tri._a.x();
                point.y = tri._a.y();
                point.z = tri._a.z();
                starMarker.points.push_back(point);
                point.x = tri._b.x();
                point.y = tri._b.y();
                point.z = tri._b.z();
                starMarker.points.push_back(point);
                point.x = tri._c.x();
                point.y = tri._c.y();
                point.z = tri._c.z();
                starMarker.points.push_back(point);
            }

            edgeMarker = starMarker;
            edgeMarker.id = static_cast<int>(idx * 2 + 1);
            edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
            edgeMarker.ns = "star_polytope_edge";
            edgeMarker.color.r = color_(0);
            edgeMarker.color.g = 0;
            edgeMarker.color.b = color_(2);
            edgeMarker.color.a = 0.5;
            edgeMarker.scale.x = 0.02;
            edgeMarker.points.reserve(tris * 6);

            for (const auto &tri : triangles)
            {
                const Eigen::Vector3d p0 = tri._a;
                const Eigen::Vector3d p1 = tri._b;
                const Eigen::Vector3d p2 = tri._c;
                point.x = p0.x();
                point.y = p0.y();
                point.z = p0.z();
                edgeMarker.points.push_back(point);
                point.x = p1.x();
                point.y = p1.y();
                point.z = p1.z();
                edgeMarker.points.push_back(point);
                point.x = p1.x();
                point.y = p1.y();
                point.z = p1.z();
                edgeMarker.points.push_back(point);
                point.x = p2.x();
                point.y = p2.y();
                point.z = p2.z();
                edgeMarker.points.push_back(point);
                point.x = p2.x();
                point.y = p2.y();
                point.z = p2.z();
                edgeMarker.points.push_back(point);
                point.x = p0.x();
                point.y = p0.y();
                point.z = p0.z();
                edgeMarker.points.push_back(point);
            }

            marker_array.markers.push_back(starMarker);
            marker_array.markers.push_back(edgeMarker);
        }

        starPub.publish(marker_array);
    }

private:
    // shared helper: build a mesh from triangle vertices and publish to a mesh/edge publisher pair
    inline void visualizeMeshOn(const Eigen::Matrix3Xd &tri_vertices,
                                ros::Publisher &pub_mesh,
                                ros::Publisher &pub_edge)
    {
        visualization_msgs::Marker meshMarker, edgeMarker;

        meshMarker.id = 0;
        meshMarker.header.stamp = ros::Time::now();
        meshMarker.header.frame_id = frame_id_;
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
        int ptnum = tri_vertices.cols();

        for (int i = 0; i < ptnum; i++)
        {
            point.x = tri_vertices(0, i);
            point.y = tri_vertices(1, i);
            point.z = tri_vertices(2, i);
            meshMarker.points.push_back(point);
        }

        for (int i = 0; i < ptnum / 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                point.x = tri_vertices(0, 3 * i + j);
                point.y = tri_vertices(1, 3 * i + j);
                point.z = tri_vertices(2, 3 * i + j);
                edgeMarker.points.push_back(point);
                point.x = tri_vertices(0, 3 * i + (j + 1) % 3);
                point.y = tri_vertices(1, 3 * i + (j + 1) % 3);
                point.z = tri_vertices(2, 3 * i + (j + 1) % 3);
                edgeMarker.points.push_back(point);
            }
        }

        pub_mesh.publish(meshMarker);
        pub_edge.publish(edgeMarker);
    }
};

#endif
