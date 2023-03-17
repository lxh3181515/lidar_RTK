#include "lidar_RTK/models/optimizer/optimizer_g2o.hpp"


OptimizerG2O::OptimizerG2O() {
    solver_ptr_ = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    optimizer_ptr_ = new g2o::SparseOptimizer();
    optimizer_ptr_->setAlgorithm(solver_ptr_);

    node_cnt_ = 0;
    edge_cnt_ = 0;
    optimize_times_ = 30;
}


int OptimizerG2O::getNodeNum() {
    return optimizer_ptr_->vertices().size();
}


bool OptimizerG2O::addSE3Node(const Eigen::Isometry3d &estimated_pose, bool need_fix) {
    g2o::VertexSE3 * v = new g2o::VertexSE3();
    v->setId(node_cnt_++);
    v->setEstimate(estimated_pose);
    if (need_fix == true && node_cnt_ == 1)
        v->setFixed(true);
    optimizer_ptr_->addVertex(v);

    return true;
}


bool OptimizerG2O::addSE3Edge(int vertex_index1,
                              int vertex_index2,
                              const Eigen::Isometry3d &measured_pose,
                              const Eigen::VectorXd noise) {
    if (vertex_index1 < 0 || vertex_index1 >= node_cnt_ || 
        vertex_index2 < 0 || vertex_index2 >= node_cnt_)
        return false;

    // 计算信息矩阵
    Eigen::MatrixXd information_matrix = calInformationMatrix(noise);

    g2o::EdgeSE3 * e = new g2o::EdgeSE3();
    e->setId(edge_cnt_++);
    e->setVertex(0, optimizer_ptr_->vertices()[vertex_index1]);
    e->setVertex(1, optimizer_ptr_->vertices()[vertex_index2]);
    e->setInformation(information_matrix);
    e->setMeasurement(measured_pose);
    optimizer_ptr_->addEdge(e);

    return true;
}


bool OptimizerG2O::addSE3PriorXYZEdge(int vertex_index,
                                    const Eigen::Vector3d &measured_t,
                                    const Eigen::VectorXd noise) {
    if (vertex_index < 0 || vertex_index >= node_cnt_)
        return false;

    // 计算信息矩阵
    Eigen::MatrixXd information_matrix = calInformationMatrix(noise);

    EdgeSE3PriorXYZ * e = new EdgeSE3PriorXYZ();
    e->setVertex(0, optimizer_ptr_->vertices()[vertex_index]);
    e->setInformation(information_matrix);
    e->setMeasurement(measured_t);
    optimizer_ptr_->addEdge(e);

    return true;
}


Eigen::MatrixXd OptimizerG2O::calInformationMatrix(const Eigen::VectorXd vector) {
    Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(vector.rows(), vector.rows());
    for (int i = 0; i < vector.rows(); i++) {
        information_matrix(i, i) /= vector(i);
    }
    return information_matrix;
}


bool OptimizerG2O::optimize() {
    if (optimizer_ptr_->edges().size() < 1) {
        return false;
    }

    optimizer_ptr_->initializeOptimization();
    optimizer_ptr_->computeInitialGuess();
    optimizer_ptr_->computeActiveErrors();
    optimizer_ptr_->setVerbose(false);
    optimizer_ptr_->optimize(optimize_times_);

    return true;
}


bool OptimizerG2O::getOptimizedPoses(std::deque<Eigen::Matrix4f>& optimized_poses) {
    optimized_poses.clear();
    
    int num = optimizer_ptr_->vertices().size();
    for (int i = 0; i < num; i++) {
        g2o::VertexSE3 * v = dynamic_cast<g2o::VertexSE3 *>(optimizer_ptr_->vertices()[i]);
        Eigen::Isometry3d pose = v->estimate();
        optimized_poses.push_back(pose.matrix().cast<float>());
    }
    return true;
}
