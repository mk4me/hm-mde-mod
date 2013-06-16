#ifndef _OSG_QUAT_UTILS_
#define _OSG_QUAT_UTILS_

#include <osg/Quat>
#include <type_traits>

namespace osg {

	//SIGNUM FUNCTION TAKEN FROM: http://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c 

	template <typename T>
	static inline char signum(T x, std::false_type is_signed) {
		return T(0) < x;
	}

	template <typename T>
	static inline char signum(T x, std::true_type is_signed) {
		return (T(0) < x) - (x < T(0));
	}

	template <typename T>
	static inline char signum(T x) {
		return signum(x, std::is_signed<T>());
	}

	class QuatUtils {

	public:

		static const Quat UNIT_QUAT;
		static const Quat::value_type QUAT_EPSILON;

		static const Quat exp(const Quat & val);    

		static const Quat log(const Quat & val);    

		static const Quat pow(const Quat & base, const Quat::value_type power);    

		static const Quat normalize(const Quat & val);    

		static const Quat lerp(const Quat &q1, const Quat &q2, const Quat::value_type t);    

		static const Quat slerp(const Quat &q1, const Quat &q2, const Quat::value_type t);    

		static const Quat slerpNoInvert(const Quat &from, const Quat &to, const Quat::value_type t);
    
		//! spherical cubic interpolation
		static const Quat squad(const Quat &q1,const Quat &q2,const Quat &a,const Quat &b, const Quat::value_type t);
    
		static const Quat simpleSquadCall(const Quat &q1,const Quat &q2,const Quat &q3,const Quat &q4, const Quat::value_type t);
    
		//! Shoemake-Bezier interpolation using De Castlejau algorithm
		static const Quat bezier(const Quat &q1, const Quat &q2, const Quat &a, const Quat &b, const Quat::value_type t);
    
		static const Quat simpleBezierCall(const Quat &q1,const Quat &q2,const Quat &q3,const Quat &q4, const Quat::value_type t);
    
		//! Given 3 quaternions, qn-1,qn and qn+1, calculate a control point to be used in spline interpolation
		static const Quat spline(const Quat &qnm1, const Quat &qn, const Quat &qnp1);
    
		//! Konwertuje kwaternion do k¹tów Eulera
		//! \param x
		//! \param y
		//! \param z
		//! \param w
		//! \param[out] roll
		//! \param[out] pitch
		//! \param[out] yaw
		static void quaterionToEuler(const double x, const double y,
			const double z, const double w, double & roll, double & pitch,
			double & yaw);

		//! Mwtoda konwertuje k¹ty Eulera do kwaternionu
		//! \param roll
		//! \param pitch
		//! \param yaw
		//! \param[out] x
		//! \param[out] y
		//! \param[out] z
		//! \param[out] w
		static void eulerToQuaternion(const double roll, const double pitch,
			const double yaw, double & x, double & y, double & z, double & w);
		
		template<class QuatT, class It>
		static void average(const It s, const It e, QuatT & averageQuat,
			const double minAngleChange = 0.05,
			const unsigned int maxIterations = 1000){

			if(s == e){
				throw std::runtime_error("Empty collection");
			}

			if(minAngleChange <= 0.0 || minAngleChange >= 0.5){
				throw std::runtime_error("Wrong minimum angle change value for stop criteria");
			}

			const auto size = std::distance(s, e);

			averageQuat = *s;

			QuatT adjustmentValue;
			adjustmentValue.x() = adjustmentValue.y() = adjustmentValue.z() = 0.0;
			adjustmentValue.w() = 0.5;

			const double upperBound = 1.0 - minAngleChange;

			unsigned int i = 0;	
			while(adjustmentValue.w() < upperBound && adjustmentValue.w() > minAngleChange && i++ < maxIterations){

				osg::Vec3d mean(0.0, 0.0, 0.0);

				auto meanInv = averageQuat.inverse();
				unsigned int count = 0;
				for(auto it = s; it != e; ++it){
					QuatT diff(*s * meanInv);
					double angle = 2.0 * std::acos(diff.w());

					if(angle > 0.0001){
						mean += osg::Vec3d(diff.x(), diff.y(), diff.z()) * (angle / std::sin(angle));
						++count;
					}
				}

				if(count == 0){
					break;
				}

				mean /= count;

				const auto l2 = mean.length2();

				if(l2 < 0.01){
					break;
				}
				
				const auto l = std::sqrt(l2);
				const double a2 = l / 2.0;		
				const auto axis = mean * (std::sin(a2) / l);								

				adjustmentValue.w() = std::cos(a2);
				adjustmentValue.x() = axis.x();
				adjustmentValue.y() = axis.y();
				adjustmentValue.z() = axis.z();

				averageQuat *= adjustmentValue;
			}
		}

	};

}

#endif