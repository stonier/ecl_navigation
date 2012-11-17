/**
 * @file include/ecl/slam/slam_ekf/odometry_generic.hpp
 *
 * @brief A generic odometry model for the slam-ekf.
 *
 * @date February 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_SLAM_ODOMETRY_GENERIC_HPP_
#define ECL_SLAM_ODOMETRY_GENERIC_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/linear_algebra.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace slam_ekf {

/*****************************************************************************
** Interfaces
*****************************************************************************/

template <typename FilterBase>
class GenericOdometryModel {
public:
	static const unsigned int measurement_dimension = FilterBase::measurement_dimension;
	static const unsigned int pose_dimension = FilterBase::pose_dimension;

	typedef typename FilterBase::Measurement Measurement;
	typedef typename FilterBase::Pose Pose;
	typedef typename FilterBase::OdometryMotionJacobian MotionJacobian;
	typedef typename FilterBase::OdometryNoiseJacobian NoiseJacobian;
	typedef typename FilterBase::OdometryNoise Noise;

	virtual void updateFilter(const Measurement &incoming) {

		/******************************************
		** Calculations
		*******************************************/
		Pose current_pose = filter_base.state.block(0,0,pose_dimension,1);
		MotionJacobian motion_jacobian = motionJacobian(incoming);
		NoiseJacobian noise_jacobian = noiseJacobian(incoming);
		ecl::linear_algebra::MatrixXd pose_lm_covariances;
		pose_lm_covariances = motion_jacobian*filter_base.covariance.block(0,pose_dimension,pose_dimension,filter_base.covariance.cols()-pose_dimension);

		/******************************************
		** Updates
		*******************************************/
		filter_base.state.block(0,0,pose_dimension,1) += poseUpdate(incoming);
		filter_base.covariance.block(0,0,pose_dimension,pose_dimension) =  motion_jacobian*(filter_base.covariance.block(0,0,pose_dimension,pose_dimension))*motion_jacobian.transpose()
						+ noise_jacobian*noise(incoming)*noise_jacobian.transpose();
		filter_base.covariance.block(0,pose_dimension,pose_dimension,filter_base.covariance.cols()-pose_dimension) = pose_lm_covariances;
		filter_base.covariance.block(pose_dimension,0,filter_base.covariance.cols()-pose_dimension,pose_dimension) = pose_lm_covariances.transpose();
	}

	Pose pose() {
		return filter_base.state.block(0,0,pose_dimension,1);
	}

protected:
	GenericOdometryModel(FilterBase &ekf_arrays) : filter_base(ekf_arrays) {}
	virtual ~GenericOdometryModel() {}

	virtual Pose poseUpdate(const Measurement &incoming ) = 0;
	virtual MotionJacobian motionJacobian(const Measurement &  incoming ) = 0;
	virtual NoiseJacobian noiseJacobian(const Measurement & incoming ) = 0;
	virtual Noise noise(const Measurement &incoming) = 0;

	FilterBase &filter_base;
};


} // namespace slam_ekf
} // namespace ecl

#endif /* ECL_SLAM_ODOMETRY_GENERIC_HPP_ */
