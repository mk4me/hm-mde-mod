//
//
//  @ Solution: ImuFilters_FrameWork
//  @ Project : ImuFilters_Lib
//  @ File Name : CAQKfFilter.cpp
//  @ Date : 2014-11-18
//  @ Author : Przemys³aw Pruszowski, Kamil Lebek
//
//

#include "CAQKfFilter.h"

//! Global namespace
namespace ImuFilters
{

	//! Internal types and implementations
	namespace Internal
	{
		//! "Infinite" value marker (not an actual infinity marker as in float number)
		const double CAQKfFilter::INFINITY_MARKER = 1000.0;

		//! Default data sampling frequency for sensor fusion
		const double CAQKfFilter::DEF_SAMPLE_FREQ = 1.0 / 100.0;

		//! Number of "uncertain" frames - call estimate() at least UNCERTAIN_FRAMES times
		const unsigned int CAQKfFilter::UNCERTAIN_FRAMES = 10;

		//! Prevent direct instantiation
		CAQKfFilter::CAQKfFilter(void)
		{
			// Reset filter state
			reset();
		}

		//! Simple destructor
		CAQKfFilter::~CAQKfFilter(void)
		{
		}

		//! Calculates orientation for the first frame
		Eigen::Quaterniond CAQKfFilter::estimate_first(const Eigen::Vector3d& inAcc, const Eigen::Vector3d& inMag, const double inDeltaT)
		{
			Utils::Matrix32 C, D;

			// Delta T - time between data acquisitions in seconds
			if (inDeltaT > 0.0)
				_Ts = inDeltaT;
			else
				_Ts = DEF_SAMPLE_FREQ;

			// Initialize matrices
			C.col(0) = inMag.normalized();
			C.col(1) = inAcc.normalized();
			
			Eigen::Vector3d MN(_MagN.x, _MagN.y, _MagN.z);
			Eigen::Vector3d GN(_GravN.x, _GravN.y, _GravN.z);
			
			D.col(0) = MN.normalized();
			D.col(1) = GN.normalized();
			
			// Use Wahba's algorithm
			Eigen::Quaterniond Q0 = Utils::Wahba(C, D);
			
			// Initialize internal state variables
			_P.setIdentity(); 
			_Pkr.setIdentity();
			_x.putQ(Q0);
			
			// Change flag
			_IsFirst = false;
			
			// Return estimated orientation
			return Q0;
		}

		//! Calculates orientation from sensor fusion
		/*!
			\param inAcc accelerometer vector from IMU
			\param inGyro gyroscope vector from IMU
			\param inMag magnetometer vector from IMU
			\return Returns estimated orientation.
		*/
		osg::Quat CAQKfFilter::estimate(const osg::Vec3& inAcc, const osg::Vec3& inGyro, const osg::Vec3& inMag, const double inDeltaT)
		{
			// Convert to JS³upik internal library
			JS::Point3D akc(inAcc.x(), inAcc.y(), inAcc.z()), 
						mag(inMag.x(), inMag.y(), inMag.z()), 
						zyr(inGyro.x(), inGyro.y(), inGyro.z());
			JS::Quaternion outQ;
			
			// Better one cache miss, than lazy function pointer to perform one-time init
			if (_IsFirst)
			{
				// Convert arguments
				const Eigen::Vector3d firstAcc(inAcc.x(), inAcc.y(), inAcc.z());
				const Eigen::Vector3d firstMag(inMag.x(), inMag.y(), inMag.z());

				// Init once
				const Eigen::Quaterniond firstQ = estimate_first(firstAcc, firstMag, inDeltaT);

				// Convert output
				return osg::Quat(firstQ.x(), firstQ.y(), firstQ.z(), firstQ.w());
			}

			// Delta T - time between data acquisitions in seconds
			if (inDeltaT > 0.0)
				_Ts = inDeltaT;
			else
				_Ts = DEF_SAMPLE_FREQ;

			// Actual orientation estimation steps
			_Fi.setNewOmega(zyr, _Ts);
			_xkr = _x.transpose() * _Fi ;
			outQ = _xkr.getQ();
			//----------
			setQMatrixFromQ(_Q, outQ);
			_Pkr = _Fi * _P * _Fi.transpose() + _Q;
			//----------
			calculateJacobian(_F, _xkr);
			setRMatrix(_R, akc, mag);
			_TMP = _R + _F * _Pkr * _F.transpose();
			_K = _Pkr * _F.transpose() * _TMP.inverse();
			//----------
			_z << akc.x, akc.y, akc.z, mag.x, mag.y, mag.z;
			measurementFunction(_zOut, _xkr);		
			_zTmp = _z - _zOut; 
			//----------------
			_x = _xkr + _K * _zTmp;
  			outQ = _x.getQ();
			outQ = (1.0 / outQ.Qnorm()) * outQ; // Normalization here is a must!
			_x.putQ(outQ);
			//----------
			_P = _Pkr - _K * _F * _Pkr;
			//----------
			
			// Return estimated orientation
			// Slupik's Quaternion (a, b, c, d) = Eigen's Quaternion(w, x, y, z)
			// osg's Quaternion (x, y, z, w)
			return osg::Quat(outQ.b, outQ.c, outQ.d, outQ.a);
		}

		//! Converts quaternion to a matrix
		void CAQKfFilter::setQMatrixFromQ(QMatrix &outMatrix, const JS::Quaternion& inQuat) const
		{
			double w = (_Ts * _Ts * _sigma2w) / 4.0;
			outMatrix <<	w *(inQuat.a * inQuat.a + inQuat.d * inQuat.d + inQuat.c * inQuat.c), -w * inQuat.b * inQuat.c, -w * inQuat.b * inQuat.d, -w * inQuat.b * inQuat.a, 
							-w * inQuat.c * inQuat.b, w * (inQuat.a * inQuat.a + inQuat.d * inQuat.d + inQuat.b * inQuat.b), -w * inQuat.c * inQuat.d, -w * inQuat.c * inQuat.a, 
							-w * inQuat.d * inQuat.b, -w * inQuat.d * inQuat.c, w * (inQuat.a * inQuat.a + inQuat.c * inQuat.c + inQuat.b * inQuat.b), -w * inQuat.d * inQuat.a, 
							-w * inQuat.a * inQuat.b, -w * inQuat.a * inQuat.c, -w * inQuat.a * inQuat.d, w * (inQuat.b * inQuat.b + inQuat.c * inQuat.c + inQuat.d * inQuat.d);
		}

		//! Calculates Jacobia from the given state vector
		void CAQKfFilter::calculateJacobian(FMatrix &outMatrix, const JS::StateVector& inStateVector) const
		{
			// Caculate norm using x(i,0) as q_i
			double odNorma = 1.0 / sqrt(inStateVector(0, 0) * inStateVector(0, 0) 
							+ inStateVector(1, 0) * inStateVector(1, 0) 
							+ inStateVector(2, 0) * inStateVector(2, 0) 
							+ inStateVector(3, 0) * inStateVector(3, 0));
			
			// If we implement it as in article, it should be equal to 1
			double odNorma2 = odNorma; 

			// Reset output
			outMatrix.setZero();
       
			// Initialize DC matrix
			JS::DCMatrix C(JS::Quaternion(inStateVector(3, 0), inStateVector(0, 0), inStateVector(1, 0), inStateVector(2, 0)));
   
			// pocMagNodna using q1 -> x(0,0)
			outMatrix(0, 0) = odNorma * ((2.0 * inStateVector(0, 0) - odNorma2 * inStateVector(0, 0) * C(0, 0)) * _GravN.x +
										(2.0 * inStateVector(1, 0) - odNorma2 * inStateVector(0, 0) * C(0, 1)) * _GravN.y +
										(2.0 * inStateVector(2, 0) - odNorma2 * inStateVector(0, 0) * C(0, 2)) * _GravN.z);

			outMatrix(1, 0) = odNorma * ((2.0 * inStateVector(1, 0) - odNorma2 * inStateVector(0, 0) * C(1, 0)) * _GravN.x +
										(-2.0 * inStateVector(0, 0) - odNorma2 * inStateVector(0, 0) * C(1, 1)) * _GravN.y +
										(2.0 * inStateVector(3, 0) - odNorma2 * inStateVector(0, 0) * C(1, 2)) * _GravN.z);
			
			outMatrix(2, 0) = odNorma * ((2.0 * inStateVector(2, 0) - odNorma2 * inStateVector(0, 0) * C(2, 0)) * _GravN.x +
										(-2.0 * inStateVector(3, 0) - odNorma2 * inStateVector(0, 0) * C(2, 1)) * _GravN.y +
										(-2.0 * inStateVector(0, 0) - odNorma2 * inStateVector(0, 0) * C(2, 2)) * _GravN.z);
			
			outMatrix(3, 0) = odNorma * ((2.0 * inStateVector(0, 0) - odNorma2 * inStateVector(0, 0) * C(0, 0)) * _MagN.x +
										(2.0 * inStateVector(1, 0) - odNorma2 * inStateVector(0, 0) * C(0, 1)) * _MagN.y +
										(2.0 * inStateVector(2, 0) - odNorma2 * inStateVector(0, 0) * C(0, 2)) * _MagN.z);
			
			outMatrix(4, 0) = odNorma * ((2.0 * inStateVector(1, 0) - odNorma2 * inStateVector(0, 0) * C(1, 0)) * _MagN.x +
										(-2.0 * inStateVector(0, 0) - odNorma2 * inStateVector(0, 0) * C(1, 1)) * _MagN.y +
										(2.0 * inStateVector(3, 0) - odNorma2 * inStateVector(0, 0) * C(1, 2)) * _MagN.z);
			
			outMatrix(5, 0) = odNorma * ((2.0 * inStateVector(2, 0) - odNorma2 * inStateVector(0, 0) * C(2, 0)) * _MagN.x +
										(-2.0 * inStateVector(3, 0) - odNorma2 * inStateVector(0, 0) * C(2, 1)) * _MagN.y +
										(-2.0 * inStateVector(0, 0) - odNorma2 * inStateVector(0, 0) * C(2, 2)) * _MagN.z);
			
			// poc_MagNodna using q2  - > x(1, 0)
			outMatrix(0, 1) = odNorma * ((-2.0 * inStateVector(1, 0) - odNorma2 * inStateVector(1, 0) * C(0, 0)) * _GravN.x +
										(2.0 * inStateVector(0, 0) - odNorma2 * inStateVector(1, 0) * C(0, 1)) * _GravN.y +
										(-2.0 * inStateVector(3, 0) - odNorma2 * inStateVector(1, 0) * C(0, 2)) * _GravN.z);

			outMatrix(1, 1) = odNorma * ((2.0 * inStateVector(0, 0) - odNorma2 * inStateVector(1, 0) * C(1, 0)) * _GravN.x +
										(2.0 * inStateVector(1, 0) - odNorma2 * inStateVector(1, 0) * C(1, 1)) * _GravN.y +
										(2.0 * inStateVector(2, 0) - odNorma2 * inStateVector(1, 0) * C(1, 2)) * _GravN.z);

			outMatrix(2, 1) = odNorma * ((2.0 * inStateVector(3, 0) - odNorma2 * inStateVector(1, 0) * C(2, 0)) * _GravN.x +
										(2.0 * inStateVector(2, 0) - odNorma2 * inStateVector(1, 0) * C(2, 1)) * _GravN.y +
										(-2.0 * inStateVector(1, 0) - odNorma2 * inStateVector(1, 0) * C(2, 2)) * _GravN.z);

			outMatrix(3, 1) = odNorma * ((-2.0 * inStateVector(1, 0) - odNorma2 * inStateVector(1, 0) * C(0, 0)) * _MagN.x +
										(2.0 * inStateVector(0, 0) - odNorma2 * inStateVector(1, 0) * C(0, 1)) * _MagN.y +
										(-2.0 * inStateVector(3, 0) - odNorma2 * inStateVector(1, 0) * C(0, 2)) * _MagN.z);

			outMatrix(4, 1) = odNorma * ((2.0 * inStateVector(0, 0) - odNorma2 * inStateVector(1, 0) * C(1, 0)) * _MagN.x +
										(2.0 * inStateVector(1, 0) - odNorma2 * inStateVector(1, 0) * C(1, 1)) * _MagN.y +
										(2.0 * inStateVector(2, 0) - odNorma2 * inStateVector(1, 0) * C(1, 2)) * _MagN.z);

			outMatrix(5, 1) = odNorma * ((2.0 * inStateVector(3, 0) - odNorma2 * inStateVector(1, 0) * C(2, 0)) * _MagN.x +
										(2.0 * inStateVector(2, 0) - odNorma2 * inStateVector(1, 0) * C(2, 1)) * _MagN.y +
										(-2.0 * inStateVector(1, 0) - odNorma2 * inStateVector(1, 0) * C(2, 2)) * _MagN.z);

			// poc_MagNodna using q3  - > x(2, 0)
			outMatrix(0, 2) = odNorma * ((-2.0 * inStateVector(2, 0) - odNorma2 * inStateVector(2, 0) * C(0, 0)) * _GravN.x +
										(2.0 * inStateVector(3, 0) - odNorma2 * inStateVector(2, 0) * C(0, 1)) * _GravN.y +
										(2.0 * inStateVector(0, 0) - odNorma2 * inStateVector(2, 0) * C(0, 2)) * _GravN.z);

			outMatrix(1, 2) = odNorma * ((-2.0 * inStateVector(3, 0) - odNorma2 * inStateVector(2, 0) * C(1, 0)) * _GravN.x +
										(-2.0 * inStateVector(2, 0) - odNorma2 * inStateVector(2, 0) * C(1, 1)) * _GravN.y +
										(2.0 * inStateVector(1, 0) - odNorma2 * inStateVector(2, 0) * C(1, 2)) * _GravN.z);

			outMatrix(2, 2) = odNorma * ((2.0 * inStateVector(0, 0) - odNorma2 * inStateVector(2, 0) * C(2, 0)) * _GravN.x +
										(2.0 * inStateVector(1, 0) - odNorma2 * inStateVector(2, 0) * C(2, 1)) * _GravN.y +
										(2.0 * inStateVector(2, 0) - odNorma2 * inStateVector(2, 0) * C(2, 2)) * _GravN.z);

			outMatrix(3, 2) = odNorma * ((-2.0 * inStateVector(2, 0) - odNorma2 * inStateVector(2, 0) * C(0, 0)) * _MagN.x +
										(2.0 * inStateVector(3, 0) - odNorma2 * inStateVector(2, 0) * C(0, 1)) * _MagN.y +
										(2.0 * inStateVector(0, 0) - odNorma2 * inStateVector(2, 0) * C(0, 2)) * _MagN.z);

			outMatrix(4, 2) = odNorma * ((-2.0 * inStateVector(3, 0) - odNorma2 * inStateVector(2, 0) * C(1, 0)) * _MagN.x +
										(-2.0 * inStateVector(2, 0) - odNorma2 * inStateVector(2, 0) * C(1, 1)) * _MagN.y +
										(2.0 * inStateVector(1, 0) - odNorma2 * inStateVector(2, 0) * C(1, 2)) * _MagN.z);

			outMatrix(5, 2) = odNorma * ((2.0 * inStateVector(0, 0) - odNorma2 * inStateVector(2, 0) * C(2, 0)) * _MagN.x +
										(2.0 * inStateVector(1, 0) - odNorma2 * inStateVector(2, 0) * C(2, 1)) * _MagN.y +
										(2.0 * inStateVector(2, 0) - odNorma2 * inStateVector(2, 0) * C(2, 2)) * _MagN.z);

			// poc_MagNodna using q4  - > x(3, 0)
			outMatrix(0, 3) = odNorma * ((2.0 * inStateVector(3, 0) - odNorma2 * inStateVector(3, 0) * C(0, 0)) * _GravN.x +
										(2.0 * inStateVector(2, 0) - odNorma2 * inStateVector(3, 0) * C(0, 1)) * _GravN.y +
										(-2.0 * inStateVector(1, 0) - odNorma2 * inStateVector(3, 0) * C(0, 2)) * _GravN.z);

			outMatrix(1, 3) = odNorma * ((-2.0 * inStateVector(2, 0) - odNorma2 * inStateVector(3, 0) * C(1, 0)) * _GravN.x +
										(2.0 * inStateVector(3, 0) - odNorma2 * inStateVector(3, 0) * C(1, 1)) * _GravN.y +
										(2.0 * inStateVector(0, 0) - odNorma2 * inStateVector(3, 0) * C(1, 2)) * _GravN.z);

			outMatrix(2, 3) = odNorma * ((2.0 * inStateVector(1, 0) - odNorma2 * inStateVector(3, 0) * C(2, 0)) * _GravN.x +
										(-2.0 * inStateVector(0, 0) - odNorma2 * inStateVector(3, 0) * C(2, 1)) * _GravN.y +
										(2.0 * inStateVector(3, 0) - odNorma2 * inStateVector(3, 0) * C(2, 2)) * _GravN.z);

			outMatrix(3, 3) = odNorma * ((2.0 * inStateVector(3, 0) - odNorma2 * inStateVector(3, 0) * C(0, 0)) * _MagN.x +
										(2.0 * inStateVector(2, 0) - odNorma2 * inStateVector(3, 0) * C(0, 1)) * _MagN.y +
										(-2.0 * inStateVector(1, 0) - odNorma2 * inStateVector(3, 0) * C(0, 2)) * _MagN.z);

			outMatrix(4, 3) = odNorma * ((-2.0 * inStateVector(2, 0) - odNorma2 * inStateVector(3, 0) * C(1, 0)) * _MagN.x +
										(2.0 * inStateVector(3, 0) - odNorma2 * inStateVector(3, 0) * C(1, 1)) * _MagN.y +
										(2.0 * inStateVector(0, 0) - odNorma2 * inStateVector(3, 0) * C(1, 2)) * _MagN.z);

			outMatrix(5, 3) = odNorma * ((2.0 * inStateVector(1, 0) - odNorma2 * inStateVector(3, 0) * C(2, 0)) * _MagN.x +
										(-2.0 * inStateVector(0, 0) - odNorma2 * inStateVector(3, 0) * C(2, 1)) * _MagN.y +
										(2.0 * inStateVector(3, 0) - odNorma2 * inStateVector(3, 0) * C(2, 2)) * _MagN.z);
		}

		//! Calculate R matrix
		void CAQKfFilter::setRMatrix(RMatrix& outMatrix, const JS::Point3D& inAcc, const JS::Point3D& inMag) const
		{
			const int ka = 3;
			const double epsilonA = 1.0;
			const double epsilonM = 1.0;
			double diagonalA = _sigma2a;
			double diagonalM = _sigma2m;
			double tmpA;
			double tmpM;
			
			tmpA = fabs(inAcc.norm() - _GravN.norm());
			if( tmpA > epsilonA )
				diagonalA = INFINITY_MARKER;
			
			tmpM = fabs(inMag.norm() - _MagN.norm());
			if( tmpM > epsilonM ) 
				diagonalM = INFINITY_MARKER;

			outMatrix <<	diagonalA, 0.0, 0.0, 0.0, 0.0, 0.0, 
							0.0, diagonalA, 0.0, 0.0, 0.0, 0.0, 
							0.0, 0.0, diagonalA, 0.0, 0.0, 0.0, 
							0.0, 0.0, 0.0, diagonalM, 0.0, 0.0, 
							0.0, 0.0, 0.0, 0.0, diagonalM, 0.0, 
							0.0, 0.0, 0.0, 0.0, 0.0, diagonalM; 
		}

		//! Measurement function for the state vector
		void CAQKfFilter::measurementFunction(ZVector& outMatrix, const JS::StateVector& inStateVector) const
		{
			JS::Quaternion q = inStateVector.getQ();
			JS::DCMatrix C(q);
			Eigen::Matrix<double, 3, 1> p1, p2, vg, vh, vBiasA, vBiasM;
		   
			vg << _GravN.x, _GravN.y, _GravN.z;
			vh << _MagN.x, _MagN.y, _MagN.z;
		   
			p1 = C * vg;
			p2 = C * vh;
		   
			outMatrix(0, 0) = p1(0, 0);
			outMatrix(1, 0) = p1(1, 0);
			outMatrix(2, 0) = p1(2, 0);
			outMatrix(3, 0) = p2(0, 0);
			outMatrix(4, 0) = p2(1, 0);
			outMatrix(5, 0) = p2(2, 0);
		}

		// Public Interface
		//! Creates an instance of given filter
		IOrientationFilter* CAQKfFilter::create()
		{
			// Self-allocation
			return new CAQKfFilter();
		}

		//! Returns internal filter name
		std::string CAQKfFilter::name() const
		{
			// Get compile-time name
			return typeid(this).name();
		}
		
		//! Resets filter internal state
		void CAQKfFilter::reset()
		{
			// Xsense parameters
			_sigma2w = 0.0001;
			_sigma2a = 0.001;
			_sigma2m = 0.00001;

			// Time difference between frames (default 100 Hz sampling rate == 1.0 / 100.0)
			_Ts = 1.0 / 100.0;
			
			// Initial values for magnetometer and gravitational vector
			_GravN = JS::Point3D(0.0, 0.0, 9.81);
			_MagN = JS::Point3D(0.4068, 0.0, -0.9135);
			
			// Reset state / temp variables
			_Fi.setIdentity();
			_P.setIdentity();
			_Pkr.setIdentity();
			_R.setIdentity();
			_TMP.setIdentity();
			_K.setIdentity();
			_F.setIdentity();
			_Q.setIdentity();
			_z.setIdentity();
			_zOut.setIdentity();
			_zTmp.setIdentity();
			_x.setIdentity();
			_xkr.setIdentity();
			_Qstate.setIdentity();

			// One time init for the first frame after reset()
			_IsFirst = true;
		}

		//! Returns number (n) of frames that are probably unstable after filter create() / reset() - call estimate() at least (n) times
		unsigned int CAQKfFilter::approximateEstimationDelay() const
		{
			// Return the default
			return UNCERTAIN_FRAMES;
		}

	} // Internal

} // ImuFilters
