/**
 * @file /src/demos/slammer1d.cpp
 *
 * @brief This is a one dimensional slammer.
 *
 * Represents a platform sliding along in one dimension with landmarks
 * on the ceiling.
 *
 * It's not really demo'ing at the moment, just running the code!
 *
 * @date February 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <string>
#include <vector>
#include <ecl/formatters.hpp>
#include <ecl/utilities/parameter.hpp>
#include <ecl/slam/slam_ekf_base.hpp>
#include <ecl/slam/slam_ekf/odometry_generic.hpp>
#include <ecl/slam/slam_ekf/observation_generic.hpp>

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;
using std::vector;
using ecl::Format;
using ecl::RightAlign;
using ecl::linear_algebra::Matrix;
using ecl::linear_algebra::MatrixXd;
using ecl::linear_algebra::Vector2d;
using ecl::linear_algebra::Vector3d;
using ecl::linear_algebra::VectorXd;
using ecl::slam_ekf::GenericOdometryModel;
using ecl::slam_ekf::GenericObservationModel;
using ecl::SlamEkfBase;
using ecl::Parameter;

/*****************************************************************************
** Classes
*****************************************************************************/

namespace ecl {
namespace demos {

/******************************************
** LandMark
*******************************************/

class LandMark {
public:
	static const unsigned int observation_dimension = 1;
	typedef Matrix<double,observation_dimension,1> Observation;

	friend class SlidingObservationModel;

	const unsigned int& id() const { return unique_id; }

	/**
	 * Observations are the bearing angle from the ground to the
	 * observed landmark - subsequently these angles have domain
	 * [0,2pi].
	 *
	 * @param obs : input observation vector.
	 */
	void observation(const Observation& obs) {
		last_observation = obs;
	}
	const Observation& observation() const { return last_observation; }


	Parameter<unsigned int> correspondence;
	~LandMark() {};

private:
	LandMark(const unsigned int& corr) :
		correspondence(corr)
	{
		static unsigned int id_counter = 0;
		unique_id = id_counter++;
	}

	Observation last_observation;
	unsigned int unique_id;
};

/******************************************
** Odometry
*******************************************/

typedef SlamEkfBase<1,1,1,2> SlidingFilterBase;

class SlidingOdometryModel : public GenericOdometryModel< SlidingFilterBase > {
public:
	SlidingOdometryModel(const double &noise_lvl, SlidingFilterBase &ekf_arrays) : GenericOdometryModel< SlidingFilterBase >(ekf_arrays), noise_level(noise_lvl) {}

private:
	Pose poseUpdate(const Measurement &incoming ) {
		Pose update = incoming;
		return update;
	}

	MotionJacobian motionJacobian(const Measurement & /* incoming */) {
		MotionJacobian jacobian;
		jacobian << 1;
		return jacobian;
	}

	NoiseJacobian noiseJacobian(const Measurement & /* incoming */) {
		NoiseJacobian noise_jacobian;
		noise_jacobian << 1;
		return noise_jacobian;
	}

	Noise noise(const Measurement &incoming) {
		Noise noise_matrix;
		noise_matrix << noise_level*fabs(incoming[0])*noise_level*fabs(incoming[0]);
		return noise_matrix;
	}

	double noise_level;
};

/******************************************
** LandMarkManager
*******************************************/

class SlidingObservationModel : public GenericObservationModel< SlidingFilterBase > {
public:
	SlidingObservationModel( SlidingFilterBase &ekf_arrays) : GenericObservationModel< SlidingFilterBase >(ekf_arrays) {
		initial_variance = Matrix<double,feature_dimension,feature_dimension>::Zero();
		for ( unsigned int i = 0; i < feature_dimension; ++i ) {
			initial_variance(i,i) = 20;
		}
	}
	LandMark createLandMark(const Vector2d &initial_state) {
		int correspondence = filter_base.insert(initial_state, initial_variance);
		return LandMark(correspondence);
	}
	void deleteLandMark(const LandMark& landmark) {
		filter_base.remove(landmark.correspondence());
	}

	Observation prediction( const Feature &feature, const Pose &current_pose ) {
		double x_r = current_pose[0];
		Observation pred;
		pred << atan2(feature[1],feature[0]-x_r);
		return pred;
	}

	Innovation innovation( const Observation &observation, const Feature &feature, const Pose &current_pose ) {
		return prediction(feature, current_pose)-observation;
	}

private:

	/**
	 * angle_predicted = arctan ( z_l / (x_l-x_r) ) = h([x_r, 0, 0, .. , x_l, z_l, 0, .., 0])
	 *
	 * The observation jacobian has two parts, the robot part (dangle/dx_r) and the
	 * landmark part (dangle/dx_l, dangle/dx_r).
	 *
	 * dangle/dx_r = z_l/(x_l-x_r)^2
	 */
	Jacobian observationJacobian( const Feature &feature, const Pose &current_pose ) {
		Jacobian observation_jacobian;
		double z_l = feature[1];
		double dx = feature[0] - current_pose[0];
		double dadxr = z_l/(z_l*z_l+dx*dx);
		double dadxl = -dadxr;
		double dadzl = dx/(z_l*z_l+dx*dx);
		observation_jacobian << dadxr, dadxl, dadzl;
		return observation_jacobian;
	}

	Noise noise() {
			static Noise noise_matrix = Noise::Zero();
			static bool noise_initialised = false;
			if ( !noise_initialised ) {
				noise_matrix << 0.001;
			}
			return noise_matrix;
	}

	FeatureVariance initial_variance;
};

} // namespace demos
} // namespace ecl

/*****************************************************************************
** Using
*****************************************************************************/

using namespace ecl::demos;

/*****************************************************************************
** Main
*****************************************************************************/

int main() {
    Format<string> string_format; string_format.width(8); string_format.align(RightAlign);
    Format<double> format; format.width(8); format.precision(2); format.align(RightAlign);

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   Construction" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    const unsigned int initial_capacity = 5;
    SlidingFilterBase filter_base(initial_capacity);

    SlidingOdometryModel odometry_model(0.1, filter_base);
    SlidingObservationModel observation_model(filter_base);

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                Adding a LandMark" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    Vector2d initial_state;
    initial_state << 0.3, 2.6;
    LandMark landmark = observation_model.createLandMark(initial_state);
    initial_state << 0.6, 2.6;
    LandMark landmark_two = observation_model.createLandMark(initial_state);
    initial_state << 0.9, 2.6;
    LandMark landmark_three = observation_model.createLandMark(initial_state);

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                 Deleting a landmark" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    observation_model.deleteLandMark(landmark_two);

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   Update Loop" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    vector<SlidingObservationModel::Observation> observations;
    vector<unsigned int> correspondences;
    SlidingOdometryModel::Measurement incoming;
    incoming << 0.1;
    for ( unsigned int i = 0; i < 10; ++i ) {
    	observations.clear();
    	correspondences.clear();
    	odometry_model.updateFilter(incoming);
    	if ( i == 4 ) {
    	    std::cout << std::endl;
    	    std::cout << "***********************************************************" << std::endl;
    	    std::cout << "                      Before" << std::endl;
    	    std::cout << "***********************************************************" << std::endl;
    	    std::cout << std::endl;
    		SlidingObservationModel::Observation obs;
    		obs << 1.65; // actual 1.648
    		observations.push_back(obs);
    		correspondences.push_back(landmark.correspondence());
    		obs << 1.42; // Actual 1.418
    		observations.push_back(obs);
    		correspondences.push_back(landmark_three.correspondence());
    		observation_model.updateFilter(observations, correspondences);
            std::cout << std::endl;
            std::cout << "***********************************************************" << std::endl;
            std::cout << "                      After" << std::endl;
            std::cout << "***********************************************************" << std::endl;
            std::cout << std::endl;
    	}
    }

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Results" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << "Pose Estimate: " << odometry_model.pose().transpose() << std::endl;

    std::cout << std::endl;

    filter_base.reserve(10);

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Passed" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    return 0;
}


