#ifndef _OSG_QUAT_UTILS_
#define _OSG_QUAT_UTILS_

#include <osg/Quat>
#include <Eigen/Dense>
#include <type_traits>

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

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
		//! \param q Kwaternion koduj¹cy orientacjê
		//! \return Wektor 3D opisuj¹cy k¹ty Eulera wg kolejnych osi: x, y, z
		template<class QuatT, class Vec3T>
		static const Vec3T quaterionToEuler(const QuatT & q);

		//! Mwtoda konwertuje k¹ty Eulera do kwaternionu
		//! \param euler K¹ty eulera
		//! \return Kwaternion odpowiadaj¹cy orientacji Eulera
		template<class QuatT, class Vec3T>
		static QuatT eulerToQuaternion(const Vec3T & euler);

		/*
		Eigen::Matrix3d rotationMatrix(const Eigen::Matrix3d & dest, const Eigen::Matrix3d & src = Eigen::Matrix3d::Identity())
		{
			return src.colPivHouseholderQr().solve(dest);
		}*/

		//! Metoda wyznacza orientacje Eulera dla 3 osi uk³adu cia³¹
		//! \param x Oœ x cia³a
		//! \param y Oœ y cia³a
		//! \param z Oœ z cia³¹
		//! \return Orientacja Eulera dla takiego u³o¿enia cia³a
		template<class Vec3IT, class Vec3OT>
		static const Vec3OT axisToEuler(const Vec3IT & x,
			const Vec3IT & y, const Vec3IT & z);
		
		template<class QuatT, class ItT>
		static void average(const ItT s, const ItT e, QuatT & averageQuat,
			const double minAngleChange, const unsigned int maxIterations);

		//! \tparam T Typ dla którego zaokr¹glamy wartoœc do danego przedzia³u
		//! \param value Wartoœæ któr¹ zaokr¹glamy do zadanego przedzia³y
		//! \param min Minimalna wartoœæ przedzia³u
		//! \param max Maksymalna wartoœæ przedzia³u
		//! \return Wartoœæ value przeskalowana modulo do zadanego przedzia³u
		template<typename T>
		static const T clamp(T value, const T min, const T max);

		//! \tparam Vec3IT Wektor wejœciowy 3D reprezentuj¹cy 3 k¹ty eulera - Roll Pitch Yaw
		//! \param euler Wektor z k¹tami eulera do przeskalowania do przedzia³u <-PI ; PI)
		template<class Vec3IT>
		static void clampEuler(Vec3IT & euler);

		//! \tparam C Kontener do którego trafi¹ lokalne rotacje pomiêdzy kolejnymi orientacjami
		//! \tparam ItT Iterator dla orientacji w reprezentacji kwaternionowej
		//! \param rotations Kontener z wynikowymi lokalnymi rotacjami
		//! \param start Iterator pocz¹tka zakresu orientacji
		//! \param end Iterator koñca zakresu orientacji
		template<class C, class ItT>
		static void convertOrientationsToRotations(C & rotations, const ItT start, const ItT end);

		//! \tparam C Kontener do którego trafi¹ orientacje na bazie kolejnych orientacji
		//! \tparam ItT Iterator dla rotacji w reprezentacji kwaternionowej		
		//! \param orientations Kontener z wynikowymi orientacjami
		//! \param start Iterator pocz¹tka zakresu rotacji
		//! \param end Iterator koñca zakresu rotacji
		//! \param startOrientation Startowa orientacja
		template<class C, class ItT>
		static void convertRotationsToOrientations(C & orientations, const ItT start,
			const ItT end, typename C::value_type startOrientation);

		//! \tparam C Kontener do którego trafi¹ orientacje na bazie kolejnych orientacji
		//! \tparam ItT Iterator dla rotacji w reprezentacji kwaternionowej		
		//! \param orientations Kontener z wynikowymi orientacjami
		//! \param start Iterator pocz¹tka zakresu rotacji
		//! \param end Iterator koñca zakresu rotacji
		//! \param startOrientation Startowa orientacja
		template<class C, class ItT>
		static void convertRotationsToOrientations(C & orientations, const ItT start,
			const ItT end);

		//! \tparam QuatT Typ reprezentuj¹cy kwaternion
		//! \param qA Kwaternion który porównujemy
		//! \param qB Kwaternion do którego porównujemy
		//! \return Ró¿nica pomiêdzy kwaternionami, addytywna, dlatego wartoœæ bezwzglêdna
		template<class QuatT>
		static const double quatDifference(const QuatT & qA, const QuatT & qB);		

		//! \tparam ItA Typ reprezentuj¹cy iterator kolekcji z kwaterninami
		//! \tparam ItB Typ reprezentuj¹cy iterator kolekcji z kwaterninami
		//! \param startSrc Iterator pocz¹tka zakresu kwaternionów które porównyjemy
		//! \param endSrc Iterator koñca zakresu kwaternionów które porównyjemy
		//! \param startDest Iterator pocz¹tka zakresu kwaternionów do których porównyjemy
		//! \param endDest Iterator koñca zakresu kwaternionów do których porównyjemy
		//! \return Ró¿nica pomiêdzy seriami kwaternionów
		template<class ItA, class ItB>
		static const std::pair<double, ItA> quatsDifference(const ItA startSrc, const ItA endSrc,
			ItB startDest, const ItB endDest);

		template<class ItA, class ItB>
		static const std::pair<double, ItA> quatsComplexDifference(const ItA startSrc, const ItA endSrc,
			ItB startDest, const ItB endDest);

		//! \tparam ItA Typ reprezentuj¹cy iterator kolekcji z kwaterninami
		//! \tparam ItB Typ reprezentuj¹cy iterator kolekcji z kwaterninami
		//! \param startSrc Iterator pocz¹tka zakresu kwaternionów które porównyjemy
		//! \param endSrc Iterator koñca zakresu kwaternionów które porównyjemy
		//! \param startDest Iterator pocz¹tka zakresu kwaternionów do których porównyjemy
		//! \param endDest Iterator koñca zakresu kwaternionów do których porównyjemy
		//! \return Offset o jaki trzeba przesun¹æ porównywan¹ seriê kwaternionów,
		//! aby sygna³ pokry³ siê z docelowym sygna³em kwaternionów
		template<class ItA, class ItB>
		static const int rotationsOffset(const ItA startSrc,
			const ItA endSrc, ItB startDest, const ItB endDest);



		//! \tparam It Iterator spod którego mo¿na wy³uskaæ kwaternion
		//! \param start Iterator pocz¹tku sekwencji odszumianej
		//! \param end Iterator koñca sekwencji odszumianej
		//! \param threshold Wartoœæ progowa, decyduj¹ca o odszumianiu (0; 0.5)
		template<class It>
		static void denoise(It start, const It end, const double threshold);
	};





	// ------------------------------- IMPLEMENTACJA --------------------------------

	template<class It>
	void QuatUtils::denoise(It start, const It end, const double threshold)
	{

		if(threshold <= 0 || threshold >= 0.5){
			throw std::invalid_argument("Invalid threshold value for denoising");
		}

		const double upperLimit = 1.0 - threshold;

		for( ; start != end; ++start){
			const double a = std::acos((*start).w()) / osg::PI;
			if(!( (a > threshold) && (a < upperLimit ) )){
				(*start).w() = 1.0;
				(*start).x() = (*start).y() = (*start).z() = 0.0;
			}
		}
	}


	template<typename T>
	const T QuatUtils::clamp(T value, const T min, const T max)
	{
		const T range = max - min;

		while(value < min){
			value += range;
		}

		while(value >= max){
			value -= range;
		}

		return value;
	}

	template<class Vec3I>
	void QuatUtils::clampEuler(Vec3I & euler)
	{
		euler.x() = clamp<double>(euler.x(), -osg::PI, osg::PI);
		euler.y() = clamp<double>(euler.y(), -osg::PI, osg::PI);
		euler.z() = clamp<double>(euler.z(), -osg::PI, osg::PI);
	}

	template<class QuatT, class Vec3T>
	static const Vec3T QuatUtils::quaterionToEuler(const QuatT & q)
		{
			return Vec3T(std::atan2(2.0 * (q.y() * q.z() + q.w() * q.x()), std::pow(q.z(), 2.0) - std::pow(q.y(), 2.0) - std::pow(q.x(), 2.0) + std::pow(q.w(), 2.0)),
				- std::asin(2.0 * (q.x() * q.z() - q.w() * q.y())),
				std::atan2(2.0 * (q.x() * q.y() + q.w() * q.z()), std::pow(q.x(), 2.0) + std::pow(q.w(), 2.0) - std::pow(q.y(), 2.0) - std::pow(q.z(), 2.0)));
		}

	//! Mwtoda konwertuje k¹ty Eulera do kwaternionu
	//! \param euler K¹ty eulera
	//! \return Kwaternion odpowiadaj¹cy orientacji Eulera
	template<class QuatT, class Vec3T>
	static QuatT QuatUtils::eulerToQuaternion(const Vec3T & euler)
	{
		Vec3T halfEuler(euler);
		halfEuler /= 2.0;
		double c1 = std::cos(halfEuler.x());
		double s1 = std::sin(halfEuler.x());
		double c2 = std::cos(halfEuler.y());
		double s2 = std::sin(halfEuler.y());
		double c3 = std::cos(halfEuler.z());
		double s3 = std::sin(halfEuler.z());

		QuatT ret;

		ret.x() = - c1 * s2 * s3 + c2 * c3 * s1;
		ret.y() = c1 * c3 * s2 + s1 * c2 * s3;
		ret.z() = c1 * c2 * s3 - s1 * c3 * s2;
		ret.w() = c1 * c2 * c3 + s1 * s2 * s3;

		//normalizuje!!
		const double l = std::sqrt(std::pow(ret.x(), 2.0) + std::pow(ret.y(), 2.0) +
			std::pow(ret.z(), 2.0) + std::pow(ret.w(), 2.0));

		ret.x() /= l;
		ret.y() /= l;
		ret.z() /= l;
		ret.w() /= l;

		return ret;
	}

	//! Metoda wyznacza orientacje Eulera dla 3 osi uk³adu cia³¹
	//! \param x Oœ x cia³a
	//! \param y Oœ y cia³a
	//! \param z Oœ z cia³¹
	//! \return Orientacja Eulera dla takiego u³o¿enia cia³a
	template<class Vec3IT, class Vec3OT>
	static const Vec3OT QuatUtils::axisToEuler(const Vec3IT & x,
		const Vec3IT & y, const Vec3IT & z)
	{
		Eigen::Matrix3d m(Eigen::Matrix3d::Zero());
			
		m(0,0) = x.x();
		m(1,0) = x.y();
		m(2,0) = x.z();
		m(0,1) = y.x();
		m(1,1) = y.y();
		m(2,1) = y.z();
		m(0,2) = z.x();
		m(1,2) = z.y();
		m(2,2) = z.z();

		Eigen::Quaterniond q(m);

		return osg::QuatUtils::quaterionToEuler
			<Eigen::Quaterniond,Vec3OT>(q);			
	}
		
	template<class QuatT, class ItT>
	static void QuatUtils::average(const ItT s, const ItT e, QuatT & averageQuat,
		const double minAngleChange, const unsigned int maxIterations){

		if(s == e){
			throw std::runtime_error("Empty collection");
		}

		if(minAngleChange <= 0.0 || minAngleChange >= 0.5){
			throw std::runtime_error("Wrong minimum angle change value for stop criteria");
		}

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

	template<class C, class ItT>
	void QuatUtils::convertOrientationsToRotations(C & rotations, const ItT start, const ItT end)
	{
		auto prevA = start;
		auto it = start;
		std::advance(it, 1);

		for( ; it != end; ++it, ++prevA){			
			rotations.push_back((*it) * (*prevA).inverse());
		}
	}

	template<class C, class ItT>
	void QuatUtils::convertRotationsToOrientations(C & orientations, const ItT start,
		const ItT end, typename C::value_type startOrientation)
	{
		orientations.push_back(startOrientation);

		for(auto it = start; it != end; ++it){
			orientations.push_back(startOrientation *= *it);
		}
	}

	template<class C, class ItT>
	void QuatUtils::convertRotationsToOrientations(C & orientations, const ItT start,
		const ItT end)
	{
		typename C::value_type q;
		q.x() = q.y() = q.z() = 0.0;
		q.w() = 1.0;

		convertRotationsToOrientations(orientations, start, end, q);
	}

	template<class QuatT>
	const double QuatUtils::quatDifference(const QuatT & qA, const QuatT & qB)
	{
		return std::fabs(std::acos((qB * qA.inverse()).w()));
	}

	template<class ItA, class ItB>
	const std::pair<double, ItA> QuatUtils::quatsDifference(const ItA startSrc, const ItA endSrc,
		ItB startDest, const ItB endDest)
	{
		std::pair<double, ItA> ret;

		ret.first = 0.0;
		ret.second = startSrc;

		while(ret.second != endSrc && startDest != endDest){				
			ret.first += quatDifference(*(ret.second), *startDest);

			++(ret.second);
			++startDest;
		}

		return ret;
	}

	template<class ItA, class ItB>
	const std::pair<double, ItA> QuatUtils::quatsComplexDifference(const ItA startSrc, const ItA endSrc,
		ItB startDest, const ItB endDest)
	{
		std::pair<double, ItA> ret;

		ret.first = 0.0;
		ret.second = startSrc;

		while(ret.second != endSrc && startDest != endDest){

			osg::Vec3 a((*ret.second).x(), (*ret.second).y(), (*ret.second).z());
			osg::Vec3 b((*startDest).x(), (*startDest).y(), (*startDest).z());

			ret.first += quatDifference(*(ret.second), *startDest) + std::fabs(std::acos((a*b) / (a.length() + b.length())));

			++(ret.second);
			++startDest;
		}

		return ret;
	}

	template<class ItA, class ItB>
	const int QuatUtils::rotationsOffset(const ItA startSrc,
		const ItA endSrc, ItB startDest, const ItB endDest)
	{
		int ret = -1;
		int offset = 0;
		double minError = std::numeric_limits<double>::max();

		auto locDestStart = startDest;

		while(locDestStart != endDest){

			auto src = startSrc;
			auto dest = locDestStart;

			const double error = rotationsDifference(src, endSrc, dest, endDest);

			if(error < minError){
				minError = error;
				ret = offset;
			}

			if(src != endSrc){
				break;
			}

			++locDestStart;
			++offset;
		}

		return ret;
	}
}

#endif