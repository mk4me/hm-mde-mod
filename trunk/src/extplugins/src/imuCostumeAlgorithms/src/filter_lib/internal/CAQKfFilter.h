//
//
//  @ Solution: ImuFilters_FrameWork
//  @ Project : ImuFilters_Lib
//  @ File Name : CAQKfFilter.h
//  @ Date : 2014-11-18
//  @ Author : Przemys³aw Pruszowski, Kamil Lebek
//
//

#if !defined(_IMUFILTERS_LIB_CAQKFFILTER_H)
#define _IMUFILTERS_LIB_CAQKFFILTER_H

// Math includes
#include <vector>
#include <Eigen/Dense>
#include "Utils.h"
#include "JS_Quaternion.h"
#include "JS_StateVector.h"
#include "JS_DCMatrix.h"
#include "JS_FiMatrix.h"

// Common filter interface
#include "../IOrientationFilter.h"

//! Global namespace
namespace ImuFilters
{

	//! Internal types and implementations
	namespace Internal
	{

		//! AQKf Kalman Filter
		class CAQKfFilter : public IOrientationFilter
		{
		private:
			//! "Infinite" value marker (not an actual infinity marker as in float number)
			static const double INFINITY_MARKER;

			//! Default data sampling frequency for sensor fusion
			static const double DEF_SAMPLE_FREQ;

			//! Number of "uncertain" frames - call estimate() at least UNCERTAIN_FRAMES times
			static const unsigned int UNCERTAIN_FRAMES;

			//! Prevent direct instantiation
			CAQKfFilter(void);

			//! Prevent copy-construction
			CAQKfFilter(const CAQKfFilter&);

			//! Prevent assignment
			CAQKfFilter& operator=(const CAQKfFilter&);    

			// PRIVATE MATRIX TYPES
			//! Covariance matrix of measurement model
			typedef Eigen::Matrix<double, 6, 6>  RMatrix;
			//! Process noise covariance matrix
			typedef Eigen::Matrix<double, 4, 4 > QMatrix;
			//! Jacobian matrix
			typedef Eigen::Matrix<double, 6, 4 > FMatrix;
			//! Measurement vector
			typedef Eigen::Matrix<double, 6, 1 > ZVector;
			//! Kalman gain matrix
			typedef Eigen::Matrix<double, 4, 6 > KMatrix;
			//! Error covariance matrix (a posteriori and a priori)
			typedef Eigen::Matrix<double, 4, 4 > PMatrix;

			//! Xsense sensor settings
			double _sigma2w, _sigma2a, _sigma2m;
			
			//! Sensor sampling frequency (default is 100Hz == 1.0 / 100.0)
			double _Ts; 

			//! Gravitational vector
			JS::Point3D _GravN; 
			//! Magnetic field vector
			JS::Point3D _MagN; 

			//! Filter state variables
			JS::FiMatrix _Fi;
			PMatrix _P, _Pkr;
			RMatrix _R, _TMP;
			KMatrix _K;
			FMatrix _F;
			QMatrix _Q;
			ZVector _z, _zOut, _zTmp;
			JS::StateVector _x, _xkr;
			Eigen::Quaterniond _Qstate;

			//! Are we calculating the first frame?
			bool _IsFirst;
			
			//! Converts quaternion to a matrix
			void setQMatrixFromQ(QMatrix& outMatrix, const JS::Quaternion& inQuat) const;
			
			//! Calculates Jacobia from the given state vector
			void calculateJacobian(FMatrix& outMatrix, const JS::StateVector& inStateVector) const;
			
			//! Calculate R matrix
			void setRMatrix(RMatrix& outMatrix, const JS::Point3D& inAcc, const JS::Point3D& inMag) const;

			//! Measurement function for the state vector
			void measurementFunction(ZVector& outMatrix, const JS::StateVector& inStateVector) const;

			//! Calculates orientation for the first frame (performs INTERNAL initialization)
			Eigen::Quaterniond estimate_first(const Eigen::Vector3d& inAcc, const Eigen::Vector3d& inMag, const double inDeltaT);

		public:
			// Aligned allocation for Eigen
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			//! Simple destructor
			virtual ~CAQKfFilter(void);

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

#endif // _IMUFILTERS_LIB_CAQKFFILTER_H
