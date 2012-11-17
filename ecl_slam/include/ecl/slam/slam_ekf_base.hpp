/**
 * @file /include/ecl/slam/slam_ekf_base.hpp
 *
 * @brief Storage container for slam-ekf problems.
 *
 * @date February 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_SLAM_EKF_BASE_HPP_
#define ECL_SLAM_EKF_BASE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <vector>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/time/duration.hpp>
#include <ecl/time/stopwatch.hpp>
#include <ecl/linear_algebra.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interfaces
*****************************************************************************/
/**
 *
 * This is mostly aligned with the theory in "Probabilistic Robotics" by
 * Sebastian Thrun & co.
 *
 * @todo Need concepts for the odometry model and measurement model. These
 * will in turn decide the specs for the ekf.
 *
 * Size is generally referred to as the number of landmarks currently stored.
 */
template<unsigned int MeasurementDimension, unsigned int PoseDimension,
		 unsigned int ObservationDimension, unsigned int FeatureDimension>
class SlamEkfBase {
public:
	static const unsigned int measurement_dimension = MeasurementDimension; /**< @brief Dimension of the odometry measurement. **/
	static const unsigned int pose_dimension = PoseDimension;               /**< @brief Dimension of the odometry pose. **/
	static const unsigned int observation_dimension = ObservationDimension; /**< @brief Dimension of the feature observation. **/
	static const unsigned int feature_dimension = FeatureDimension;         /**< @brief Dimension of the feature itself. **/

	typedef linear_algebra::Matrix<double,measurement_dimension,1> Measurement;             /**< @brief The odometry measurement vector. **/
	typedef linear_algebra::Matrix<double,pose_dimension,1> Pose;							/**< @brief The pose vector. **/
	typedef linear_algebra::Matrix<double,pose_dimension,pose_dimension> PoseVariance;		/**< @brief The pose variance matrix (pulled from the ekf matrix). **/
	typedef linear_algebra::Matrix<double,measurement_dimension,measurement_dimension> OdometryNoise; /**< @brief The odometry noise matrix. **/
	typedef linear_algebra::Matrix<double,pose_dimension,pose_dimension> OdometryMotionJacobian;		/**< @brief The odometry motion jacobian, used... **/
	typedef linear_algebra::Matrix<double,measurement_dimension,pose_dimension> OdometryNoiseJacobian;	/**< @brief The odometry noise jacobian, used... **/

	typedef linear_algebra::Matrix<double,observation_dimension,1> Observation;                          /**< @brief The observation vector. **/
	typedef linear_algebra::Matrix<double,observation_dimension,pose_dimension+feature_dimension> ObservationJacobian; /**< @brief The observation jacobian. **/
	typedef linear_algebra::Matrix<double,observation_dimension,observation_dimension> ObservationNoise; /**< @brief The observation noise matrix. **/
	typedef linear_algebra::Matrix<double,feature_dimension,1> Feature;                                  /**< @brief The feature vector. **/
	typedef linear_algebra::Matrix<double,feature_dimension,feature_dimension> FeatureVariance;          /**< @brief The feature variance matrix (pulled from the ekf matrix). **/
	typedef linear_algebra::Matrix<double,feature_dimension,feature_dimension> FeatureVarianceJacobian;  /**< @brief The feature variance jacobian, used... **/
	typedef linear_algebra::Matrix<double,feature_dimension,pose_dimension> FeatureCoVarianceJacobian;   /**< @brief The feature covariance jacobian, used... **/

	/*********************
	** Initialisation
	**********************/
	SlamEkfBase(const unsigned int &initial_capacity = 30);
	virtual ~SlamEkfBase() {};
	void init();
	void init(const unsigned int &initial_capacity);

	/*********************
	** Modding Storage
	**********************/
	int insert(const Feature &initial_state, const FeatureVariance &initial_variance) ecl_assert_throw_decl(StandardException);
	int insert(const Feature &initial_state,
			const FeatureVariance &initial_variance,
			const FeatureVarianceJacobian &variance_jacobian,
			const FeatureCoVarianceJacobian &covariance_jacobian
			) ecl_assert_throw_decl(StandardException);
	bool remove(const unsigned int &correspondence);
	void reserve(const unsigned int& new_capacity) ecl_assert_throw_decl(StandardException);
	void completeCovariance( linear_algebra::MatrixXd & cov );

	/*********************
	** Accessors
	**********************/
	Pose pose() { return state.block(0,0,pose_dimension,1); } /**< @brief Returns the current pose. **/
	Feature feature(const unsigned int &correspondence ) { return state.block(correspondence, 0, feature_dimension, 1); } /**< @brief Returns the state for the specified feature. **/
	unsigned int size() const { return number_features; } /**< @brief Returns the number of features currently stored in the map. **/

	linear_algebra::VectorXd state;
	linear_algebra::MatrixXd covariance;

private:
	unsigned int number_features;
	unsigned int capacity();                              /**< @brief Returns the maximum number of features storable in the map. **/
};

/*****************************************************************************
** Implementation [Initialisation]
*****************************************************************************/
/**
 * @brief Configures the filter with default motion and observation models.
 *
 * This sets up the storage elements and default versions of the specified
 * motion/observation models.
 */
template<unsigned int MeasurementDimension, unsigned int PoseDimension, unsigned int ObservationDimension, unsigned int FeatureDimension>
SlamEkfBase<MeasurementDimension, PoseDimension, ObservationDimension, FeatureDimension>::SlamEkfBase(const unsigned int &initial_capacity) :
	number_features(0.0)
{ init(initial_capacity); }
/**
 * @brief Initialises the underlying storage elements with the current capacity.
 *
 * Use this to reinitialise without changing the current capacity.
 */
template<unsigned int MeasurementDimension, unsigned int PoseDimension, unsigned int ObservationDimension, unsigned int FeatureDimension>
void SlamEkfBase<MeasurementDimension, PoseDimension, ObservationDimension, FeatureDimension>::init() {
	unsigned int dimension = state.rows();
	state = linear_algebra::VectorXd::Zero(dimension);
	covariance = linear_algebra::MatrixXd::Zero(dimension,dimension);
	number_features = 0;
}
/**
 * @brief Initialises the underlying storage elements.
 *
 * This is called by the constructors internally, but also
 * useful if you need to reset the filter.
 */
template<unsigned int MeasurementDimension, unsigned int PoseDimension, unsigned int ObservationDimension, unsigned int FeatureDimension>
void SlamEkfBase<MeasurementDimension, PoseDimension, ObservationDimension, FeatureDimension>::init(const unsigned int &initial_capacity) {
	unsigned int dimension = pose_dimension + initial_capacity*feature_dimension;
	state = linear_algebra::VectorXd::Zero(dimension);
	covariance = linear_algebra::MatrixXd::Zero(dimension,dimension);
	number_features = 0;
}
/*****************************************************************************
** Implementation [SlamEkfBase Mods]
*****************************************************************************/
/**
 * @brief Adds a new feature.
 *
 * This adds a new feature to the filter. Practically, it just finds an empty
 * line in the matrices and utilises that. If the filter is already full, it will
 * do one of either:
 *
 * - debug mode : throw an exception
 * - release mode : return -1 indicating failure to add.
 *
 * Note that this does not set any covariances or skew the initial variance
 * realistically. Should only use this if such rough initialisations are not
 * of importance (i.e. you have plenty of time to settle & converge).
 *
 * @param initial_state : initial mean of the new landmark.
 * @param initial_variance : initial variance of the new landmark.
 * @return int : correspondence of the addition (-1 if failed).
 *
 * @exception StandardException : throws if the filter is already full [debug mode only].
 */
template<unsigned int MeasurementDimension, unsigned int PoseDimension, unsigned int ObservationDimension, unsigned int FeatureDimension>
int SlamEkfBase<MeasurementDimension, PoseDimension, ObservationDimension, FeatureDimension>::insert(
		const linear_algebra::Matrix<double,feature_dimension,1> &initial_state,
		const linear_algebra::Matrix<double,feature_dimension,feature_dimension> &initial_variance) ecl_assert_throw_decl(StandardException) {

	ecl_assert_throw(number_features < capacity(), StandardException(LOC,MemoryError,"The filter does not have any spare storage space (use reserve() to allocate more)."));
	bool found_unused_correspondence = false;
	unsigned int index = pose_dimension;

	while ( (!found_unused_correspondence) && ( index < static_cast<unsigned int>(state.rows()) ) ) {
		if ( covariance(index,index) == 0.0 ) {
			found_unused_correspondence = true;
		} else {
			index += feature_dimension;
		}
	}
	if ( found_unused_correspondence ) {
		state.block(index,0,feature_dimension,1) = initial_state;
		covariance.block(index,index,feature_dimension,feature_dimension) = initial_variance;
		++number_features;
		return index;
	} else {
		return -1; // will only reach here in release mode (debug mode throws before it can get here)
	}
}


template<unsigned int MeasurementDimension, unsigned int PoseDimension, unsigned int ObservationDimension, unsigned int FeatureDimension>
void SlamEkfBase<MeasurementDimension, PoseDimension, ObservationDimension, FeatureDimension>::completeCovariance( linear_algebra::MatrixXd & cov )
{
	// getting matrix size
	int target_matrix_size = pose_dimension + feature_dimension*number_features;
	int current_matrix_size = covariance.rows();
	int cnt_rows(0);
	int cnt_cols(0);

	cov.resize( target_matrix_size, target_matrix_size );

	for( int i=0; i<current_matrix_size; i++ )
	{
		 if( covariance(i,i) == 0.0 )
		 {
			 continue;
		 }
		cnt_cols=0;
		for( int j=0; j<current_matrix_size; j++ )
		{
			if( covariance(j,j) == 0.0 )
			{
				continue;
			}
			else
			{
				cov(cnt_rows,cnt_cols++) = covariance(i,j);
			}
		}

		// next row
		cnt_rows += 1;
	}

	// you have complete covariance matrix without zero part
}


/**
 * @brief Adds a new feature.
 *
 * This adds a new feature to the filter. Practically, it just finds an empty
 * line in the matrices and utilises that. If the filter is already full, it will
 * do one of either:
 *
 * - debug mode : throw an exception
 * - release mode : return -1 indicating failure to add.
 *
 * @param initial_state : initial mean of the new landmark.
 * @param initial_variance : initial variance of the new landmark.
 * @param variance_jacobian : jacobian used to initialise the landmark variance
 * @param covariance_jacobian : jacobian used to initialise robot-lm and lm-lm variances.
 * @return int : correspondence of the addition (-1 if failed).
 *
 * @exception StandardException : throws if the filter is already full [debug mode only].
 */
template<unsigned int MeasurementDimension, unsigned int PoseDimension, unsigned int ObservationDimension, unsigned int FeatureDimension>
int SlamEkfBase<MeasurementDimension, PoseDimension, ObservationDimension, FeatureDimension>::insert(
		const Feature &initial_state,
		const FeatureVariance &initial_variance,
		const FeatureVarianceJacobian &variance_jacobian,
		const FeatureCoVarianceJacobian &covariance_jacobian
		) ecl_assert_throw_decl(StandardException) {

	ecl_assert_throw(number_features < capacity(), StandardException(LOC,MemoryError,"The filter does not have any spare storage space (use reserve() to allocate more)."));
	bool found_unused_correspondence = false;
	int index = pose_dimension;
	/*********************
	** Find Unused Slot
	**********************/
	while ( (!found_unused_correspondence) && ( index < state.rows() ) ) {
		if ( covariance(index,index) == 0.0 ) {
			found_unused_correspondence = true;
		} else {
			index += feature_dimension;
		}
	}
	/*********************
	** Initialise
	**********************/
	if ( found_unused_correspondence ) {
		state.block(index,0,feature_dimension,1) = initial_state;
		/*********************
		** Landmark Variance
		**********************/
		covariance.block(index,index,feature_dimension,feature_dimension) =
				covariance_jacobian*covariance.block(0,0,pose_dimension,pose_dimension)*covariance_jacobian.transpose() +
				variance_jacobian*initial_variance*variance_jacobian.transpose();

		/*********************
		** Robot-LM Covariance
		**********************/
		linear_algebra::Matrix<double,feature_dimension,pose_dimension> landmark_robot_covariance = covariance_jacobian*covariance.block(0,0,3,3);
		covariance.block(index,0,feature_dimension,pose_dimension) = landmark_robot_covariance;
		covariance.block(0,index,pose_dimension,feature_dimension) = landmark_robot_covariance.transpose();
		/*********************
		** LM-LM Covariance
		**********************/
		for ( unsigned int i = 0; i < capacity(); ++i ) {
			int idx = pose_dimension + feature_dimension*i;
			if ( idx != index ) {
				FeatureVarianceJacobian landmark_covariance = covariance_jacobian*covariance.block(0,idx,pose_dimension,feature_dimension);
				covariance.block(index,idx,feature_dimension,feature_dimension) = landmark_covariance;
				covariance.block(idx,index,feature_dimension,feature_dimension) = landmark_covariance.transpose();
			}
		}
		++number_features;
		return index;
	} else {
		return -1; // will only reach here in release mode (debug mode throws before it can get here)
	}
}

/**
 * @brief Removes feature information with the specified correspondence from the filter.
 *
 * Removes the specified landmark from the filter. In practical terms, this
 * just zeros the entries for that correspondence in both mean and variance
 * storage containers.
 *
 * @param correspondence : index of the feature to be removed.
 * @return bool : success/failure of the removal.
 *
 * @exception StandardException : throws if the filter is already full [debug mode only].
 */
template<unsigned int MeasurementDimension, unsigned int PoseDimension, unsigned int ObservationDimension, unsigned int FeatureDimension>
bool SlamEkfBase<MeasurementDimension, PoseDimension, ObservationDimension, FeatureDimension>::remove(const unsigned int &correspondence) {
	for ( unsigned int i = 0; i < feature_dimension; ++i ) {
		state[correspondence+i] = 0.0;
	}
	for ( unsigned int i = 0; i < feature_dimension; ++i ) {
		for ( int j = 0; j < covariance.cols(); ++j ) {
			covariance(correspondence+i,j) = 0.0;
			covariance(j,correspondence+i) = 0.0;
		}
	}
	--number_features;
	return true;
}
/**
 * @brief Increases the reserve capacity of the filter (in # landmarks).
 *
 * This expands the underlying matrices, zero'ing out the new columns until
 * landmarks are actually added. This is similar to the way in which std::vector
 * behaves.
 *
 * @param new_capacity : max number of landmarks to filter.
 * @exception StandardException : throws if the new size is smaller than current [debug mode only].
 */
template<unsigned int MeasurementDimension, unsigned int PoseDimension, unsigned int ObservationDimension, unsigned int FeatureDimension>
void SlamEkfBase<MeasurementDimension, PoseDimension, ObservationDimension, FeatureDimension>::reserve(const unsigned int& new_capacity) ecl_assert_throw_decl(StandardException) {

	unsigned int current_capacity = capacity();
	unsigned int current_dimension = pose_dimension+current_capacity*feature_dimension;
	unsigned int new_dimension = pose_dimension+new_capacity*feature_dimension;

	ecl_assert_throw( new_capacity > current_capacity, StandardException(LOC,InvalidInputError,"This class has a simple policy for safety, new size should be greater than the current size (unless reinitialising)."));

	state.conservativeResize(new_dimension);
	for ( unsigned int i = current_dimension; i < new_dimension; ++i ) {
		state[i] = 0.0;
	}
	covariance.conservativeResize(new_dimension,new_dimension);
	// Directly setting is faster than using covariance.block(...) = Zero... - is there a faster method?
	for ( unsigned int i = 0; i < current_dimension; ++i )  {
		for ( unsigned int j = current_dimension; j < new_dimension; ++j ) {
			covariance(i,j) = 0.0;
		}
	}
	for ( unsigned int i = current_dimension; i < new_dimension; ++i )  {
		for ( unsigned int j = 0; j < new_dimension; ++j ) {
			covariance(i,j) = 0.0;
		}
	}
}

/*****************************************************************************
** Implementation [Private]
*****************************************************************************/

template<unsigned int MeasurementDimension, unsigned int PoseDimension, unsigned int ObservationDimension, unsigned int FeatureDimension>
unsigned int SlamEkfBase<MeasurementDimension, PoseDimension, ObservationDimension, FeatureDimension>::capacity() {
	return (state.size()-pose_dimension)/feature_dimension;
}

} // namespace ecl

#endif /* ECL_SLAM_EKF_BASE_HPP_ */
