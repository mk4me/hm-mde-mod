//
//
//  @ Solution: ImuFilters_FrameWork
//  @ Project : ImuFilters_Lib
//  @ File Name : IOrientationFilter.h
//  @ Date : 2014-11-18
//  @ Author : Kamil Lebek
//
//

#if !defined(_IMUFILTERS_LIB_IORIENTATIONFILTER_H)
#define _IMUFILTERS_LIB_IORIENTATIONFILTER_H

// Includes
#include <string>
#include <osg/Vec3>
#include <osg/Quat>

// Macros
#ifndef UNREFERENCED_PARAMETER
	#define UNREFERENCED_PARAMETER(P)          (P)
#endif

//! Global namespace
namespace ImuFilters
{
	//! Generic quaternion-based orientation filter - generates orientation as a quaternion using IMU sensor fusion
	class IOrientationFilter
	{
	public:
		//! Make it polymorphic
		virtual ~IOrientationFilter() {}

		//! Filter types
		enum FilterType { FT_INSTANTENOUS_KALMAN, FT_AQKF_KALMAN };

		// Public Interface
		//! Returns internal filter name
		virtual std::string name() const = 0;
		
		//! Resets filter internal state
		virtual void reset() = 0;

		//! Returns number (n) of frames that are probably unstable after filter create() / reset() - call estimate() at least (n) times
		virtual unsigned int approximateEstimationDelay() const = 0;

		//! Calculates orientation from sensor fusion
		/*!
			\param inAcc accelerometer vector from IMU
			\param inGyro gyroscope vector from IMU
			\param inMag magnetometer vector from IMU
			\param inDeltaT time between acquisitions in seconds [s] from IMU sensor (default is 100Hz = 1 / 100)
			\return Returns estimated orientation.
		*/
		virtual osg::Quat estimate(const osg::Vec3& inAcc, const osg::Vec3& inGyro, const osg::Vec3& inMag, const double inDeltaT = (1.0 / 100.0)) = 0;
		
		// Not used
		//virtual QWidget* configurationWidget() = 0;
	};

} // ImuFilters

#endif // _IMUFILTERS_LIB_IORIENTATIONFILTER_H