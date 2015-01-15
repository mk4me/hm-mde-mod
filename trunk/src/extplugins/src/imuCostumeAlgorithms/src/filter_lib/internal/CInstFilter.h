//
//
//  @ Solution: ImuFilters_FrameWork
//  @ Project : ImuFilters_Lib
//  @ File Name : CInstFilter.h
//  @ Date : 2014-11-18
//  @ Author : Przemys³aw Pruszowski, Kamil Lebek
//
//

#if !defined(_IMUFILTERS_LIB_INST_FILTER_H)
#define _IMUFILTERS_LIB_INST_FILTER_H

// Include Eigen definitions
#include <Eigen/Dense>

// Common filter interface
#include "../IOrientationFilter.h"

//! Global namespace
namespace ImuFilters
{

	//! Internal types and implementations
	namespace Internal
	{
		//! Instantenous Kalman filter implementation
		class CInstFilter : public IOrientationFilter
		{
		
		private:
			//! Number of "uncertain" frames - call estimate() at least UNCERTAIN_FRAMES times
			static const unsigned int UNCERTAIN_FRAMES;

			//! Prevent direct instantiation
			CInstFilter(void);

			//! Prevent copy-construction
			CInstFilter(const CInstFilter&);

			//! Prevent assignment
			CInstFilter& operator=(const CInstFilter&);      

			//! Internal state variables - gravity vector
			Eigen::Vector3d _GravN;

			//! Internal state variable - magnetometer vector
			Eigen::Vector3d _MagN;

			//! Internal filter state
			Eigen::Quaterniond _State;
		
		public:
			// Aligned allocation for Eigen
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			//! Simple destructor
			virtual ~CInstFilter(void);

			// Public Interface
			//! Creates an instance of given filter
			static IOrientationFilter* create();

			//! Returns internal filter name
			std::string name() const;
		
			//! Resets filter internal state
			void reset();

			//! Returns number (n) of frames that are probably unstable after filter create() / reset() - call estimate() at least (n) times
			unsigned int approximateEstimationDelay() const;

			//! Calculates orientation from sensor fusion
			/*!
				\param inAcc accelerometer vector from IMU
				\param inGyro gyroscope vector from IMU
				\param inMag magnetometer vector from IMU
				\param inDeltaT time between acquisitions in seconds [s] from IMU sensor (default is 100Hz = 1 / 100)
				\return Returns estimated orientation.
			*/
			osg::Quat estimate(const osg::Vec3& inAcc, const osg::Vec3& inGyro, const osg::Vec3& inMag, const double inDeltaT);
		};

	} // Internal

} // ImuFilters

#endif // _IMUFILTERS_LIB_INST_FILTER_H