//
//
//  @ Solution: ImuFilters_FrameWork
//  @ Project : ImuFilters_Lib
//  @ File Name : CInstFilter.cpp
//  @ Date : 2014-11-18
//  @ Author : Przemys³aw Pruszowski, Kamil Lebek
//
//

#include "CInstFilter.h"
#include "Utils.h"

//! Global namespace
namespace ImuFilters
{

	//! Internal types and implementations
	namespace Internal
	{
		//! Number of "uncertain" frames - call estimate() at least UNCERTAIN_FRAMES times
		const unsigned int CInstFilter::UNCERTAIN_FRAMES = 5;

		//! Prevent direct instantiation
		CInstFilter::CInstFilter(void)
		{
			// Reset internal state
			reset();
		}

		//! Simple destructor
		CInstFilter::~CInstFilter(void)
		{
		}

		// Public Interface
		//! Creates an instance of given filter
		IOrientationFilter* CInstFilter::create()
		{
			// Self-allocation
			return new CInstFilter();
		}

		//! Returns internal filter name
		std::string CInstFilter::name() const
		{
			// Get compile-time name
			return typeid(this).name();
		}
		
		//! Resets filter internal state
		void CInstFilter::reset()
		{
			// Initial values for gravity vector magnetometer and state vector
			_GravN = Eigen::Vector3d(0.0, 0.0, 9.81);
			_MagN = Eigen::Vector3d(0.4068, 0.0, -0.9135);
			_State = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
		}

		//! Returns number (n) of frames that are probably unstable after filter create() / reset() - call estimate() at least (n) times
		unsigned int CInstFilter::approximateEstimationDelay() const
		{
			// Return the default
			return UNCERTAIN_FRAMES;
		}

		//! Estimates another quaternion
		osg::Quat CInstFilter::estimate(const osg::Vec3& inAcc, const osg::Vec3& inGyro, const osg::Vec3& inMag, const double inDeltaT)
		{
			UNREFERENCED_PARAMETER(inDeltaT);

			// Convert (and let compiler optimize it)
			const Eigen::Vector3d acc(inAcc.x(), inAcc.y(), inAcc.z());
			const Eigen::Vector3d gyro(inGyro.x(), inGyro.y(), inGyro.z());
			const Eigen::Vector3d mag(inMag.x(), inMag.y(), inMag.z());

			// Algorithm
			Utils::Matrix32 C, D;
			C.col(0) = mag.normalized();
			C.col(1) = acc.normalized();
			D.col(0) = _MagN.normalized();
			D.col(1) = _GravN.normalized();
			Eigen::Quaterniond Q = Utils::Wahba(C, D);
			_State = Utils::TestQuat(Q, Utils::QNegCoeff(Q), _State);

			// Convert to a common data format
			return osg::Quat(_State.x(), _State.y(), _State.z(), _State.w());
		}

	} // Internal
} // ImuFilters