/**
 * @file /include/ecl/slam/slam_ekf/observation_generic.hpp
 *
 * @brief A generic observation model for slam-ekf.
 *
 * @date Februrary 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_SLAM_OBSERVATION_GENERIC_HPP_
#define ECL_SLAM_OBSERVATION_GENERIC_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <vector>
#include <ecl/linear_algebra.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace slam_ekf {

/*****************************************************************************
** Interfaces
*****************************************************************************/
/**
 * Requires a few handles into the landmark class.
 */
template <typename FilterBase>
class GenericObservationModel {
public:
	static const unsigned int pose_dimension = FilterBase::pose_dimension;
	static const unsigned int observation_dimension = FilterBase::observation_dimension;
	static const unsigned int feature_dimension = FilterBase::feature_dimension;

	typedef typename FilterBase::Pose Pose;
	typedef typename FilterBase::Observation Observation;
	typedef typename FilterBase::Observation Innovation;
	typedef typename FilterBase::ObservationJacobian Jacobian;
	typedef typename FilterBase::ObservationNoise Noise;
	typedef typename FilterBase::Feature Feature;
	typedef typename FilterBase::FeatureVariance FeatureVariance;

	GenericObservationModel(FilterBase &ekf_arrays) : filter_base(ekf_arrays) {}
	virtual ~GenericObservationModel() {}

	/**
	 * LandMark needs:
	 *
	 * @code
	 * unsigned int correspondence();
	 * const Observation& innovation() const;
	 * @endcode
	 *
	 * Also the observation model needs the virtual methods configured.
	 *
	 * @param observations : a set of incoming observations.
	 * @param correspondences : ekf filter correspondences associated with the observations.
	 */
	virtual void updateFilter(const std::vector<Observation> &observations, const std::vector<unsigned int> &correspondences) {

		if ( observations.empty() ) { return; }
		// should also check if the observation size = corespondences size

		const unsigned int M = observations.size();

		ecl::linear_algebra::MatrixXd total_observation_jacobian = ecl::linear_algebra::MatrixXd::Zero(observation_dimension*M, filter_base.covariance.cols());
		ecl::linear_algebra::VectorXd total_innovation = ecl::linear_algebra::VectorXd::Zero(observation_dimension*M);
		ecl::linear_algebra::MatrixXd total_measurement_noise = ecl::linear_algebra::MatrixXd::Zero(observation_dimension*M, observation_dimension*M);
		Jacobian observation_jacobian;

		for ( unsigned int i = 0; i < M; ++i ) {
			unsigned int correspondence = correspondences[i];
			total_innovation.block(observation_dimension*i,0,observation_dimension,1) = innovation(observations[i],filter_base.feature(correspondence),pose());
			observation_jacobian = observationJacobian(filter_base.feature(correspondence), pose());
			total_observation_jacobian.block(observation_dimension*i,0,observation_dimension,pose_dimension) = observation_jacobian.block(0,0,observation_dimension,pose_dimension);
			total_observation_jacobian.block(observation_dimension*i,correspondence,observation_dimension,feature_dimension) = observation_jacobian.block(0,0,observation_dimension,feature_dimension);
			total_measurement_noise.block(observation_dimension*i,observation_dimension*i,observation_dimension,observation_dimension) = noise();
		}
		ecl::linear_algebra::MatrixXd total_measurement_uncertainty = total_observation_jacobian*filter_base.covariance*total_observation_jacobian.transpose()+total_measurement_noise;
		ecl::linear_algebra::MatrixXd total_gains = filter_base.covariance*total_observation_jacobian.transpose()*total_measurement_uncertainty.inverse();
		ecl::linear_algebra::VectorXd mean_updates = total_gains*total_innovation;
		filter_base.state += mean_updates;
		filter_base.covariance -= (total_gains*total_measurement_uncertainty*total_gains.transpose());
		filter_base.covariance = 0.5*(filter_base.covariance+filter_base.covariance.adjoint());
	}

	Pose pose() {
		return filter_base.state.block(0,0,pose_dimension,1);
	}

protected:
	FilterBase &filter_base;

	virtual Jacobian observationJacobian( const Feature& feature, const Pose &current_pose ) = 0;
	virtual Noise noise() = 0;
	virtual Innovation innovation( const Observation &observation, const Feature &feature, const Pose &current_pose ) = 0;

};


} // namespace slam_ekf
} // namespace ecl

#endif /* ECL_SLAM_OBSERVATION_GENERIC_HPP_ */
