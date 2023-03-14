#ifndef LIDAR_RTK_MODELS_OPTIMIZER_OPTIMIZER_HPP_
#define LIDAR_RTK_MODELS_OPTIMIZER_OPTIMIZER_HPP_

#include "Eigen/Dense"
#include <deque>

class Optimizer {
public:
    virtual ~Optimizer() = default;

    virtual bool addSE3Node(const Eigen::Isometry3d &estimated_pose, bool need_fix) = 0;
    virtual bool addSE3Edge(int vertex_index1,
                            int vertex_index2,
                            const Eigen::Isometry3d &measured_pose,
                            const Eigen::VectorXd noise) = 0;
    virtual bool addSE3PriorXYZEdge(int vertex_index,
                                    const Eigen::Vector3d &measured_pose,
                                    const Eigen::VectorXd noise) = 0;
    virtual int getNodeNum() = 0;
    virtual bool optimize() = 0;
    virtual bool getOptimizedPoses(std::deque<Eigen::Matrix4f>& optimized_poses) = 0;
};

#endif
