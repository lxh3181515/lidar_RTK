#ifndef LIDAR_RTK_MODELS_OPTIMIZER_OPTIMIZER_G2O_HPP_
#define LIDAR_RTK_MODELS_OPTIMIZER_OPTIMIZER_G2O_HPP_

#include "lidar_RTK/models/optimizer/optimizer.hpp"
#include "lidar_RTK/models/optimizer/optimizer_g2o_edge.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/factory.h>

class OptimizerG2O : public Optimizer {
public:
    OptimizerG2O();

    bool addSE3Node(const Eigen::Isometry3d &estimated_pose, bool fix) override;
    bool addSE3Edge(int vertex_index1,
                    int vertex_index2,
                    const Eigen::Isometry3d &measured_pose,
                    const Eigen::VectorXd noise) override;
    bool addSE3PriorXYZEdge(int vertex_index,
                            const Eigen::Vector3d &measured_pose,
                            const Eigen::VectorXd noise) override;
    bool optimize() override;
    bool getOptimizedPoses(std::deque<Eigen::Matrix4f>& optimized_poses) override;
    int getNodeNum() override;

    Eigen::MatrixXd calInformationMatrix(const Eigen::VectorXd vector);

private:
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;

    g2o::OptimizationAlgorithmLevenberg *solver_ptr_;
    g2o::SparseOptimizer* optimizer_ptr_;

    g2o::RobustKernelFactory *robust_kernel_factory_;

    int node_cnt_, edge_cnt_;
    int optimize_times_;
};

#endif
