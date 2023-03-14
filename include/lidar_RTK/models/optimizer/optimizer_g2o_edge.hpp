#ifndef LIDAR_RTK_MODELS_OPTIMIZER_OPTIMIZER_G2O_EDGE_HPP_
#define LIDAR_RTK_MODELS_OPTIMIZER_OPTIMIZER_G2O_EDGE_HPP_

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>


class EdgeSE3PriorXYZ : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3PriorXYZ()
    	: g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3>() {
	}

    void computeError() {
        const g2o::VertexSE3* v = static_cast<const g2o::VertexSE3*>(_vertices[0]);
        Eigen::Vector3d estimate = v->estimate().translation();

        _error = estimate - _measurement;
    }

	virtual bool read(std::istream& is) {}

	virtual bool write(std::ostream& os) const {}
};

#endif
