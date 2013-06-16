#include <QuatUtils/QuatUtils.h>
#include <cmath>
#include <algorithm>
#include <assert.h>
#include <osg/Vec3>

using namespace osg;

const double QuatUtils::QUAT_EPSILON = 0.000001;
const Quat QuatUtils::UNIT_QUAT = Quat(0.0, 0.0, 0.0, 1.0);

const Quat QuatUtils::exp(const Quat & val)
{
	assert(val.w() == 0.0);
	
    Vec3d vec(val.asVec3());
	Vec3d::value_type l = vec.length();
	double sinL = std::sin(l);

	if(sinL > QUAT_EPSILON){
		vec *= sinL / l;
	}else {
		l = vec[0] = vec[1] = vec[2] = 0.0;
	}

	return Quat(vec.x(), vec.y(), vec.z(), std::cos(l));
}

const Quat QuatUtils::log(const Quat & val)
{
	double angle = std::acos(std::min(std::max(val.w(), -1.0), 1.0));
	double sinAlpha = std::sin(angle);
	Vec3d vec(0.0, 0.0, 0.0);

	if(sinAlpha > QUAT_EPSILON){
		vec = val.asVec3() * angle / sinAlpha;
	}

	return Quat(vec.x(), vec.y(), vec.z(), 0.0);
}

const Quat QuatUtils::pow(const Quat & base, const Quat::value_type power)
{
	return exp(log(base) * power);
}

const Quat QuatUtils::normalize(const Quat & val)
{
    double l = val.length2();
    if(l == 0.0){
        return UNIT_QUAT;
    }

	return Quat(val) / std::sqrt(l);
}

const Quat QuatUtils::lerp(const Quat &q1, const Quat &q2, const Quat::value_type t)
{
	return q1 * (1.0 - t) + q2 * t;
}

const Quat QuatUtils::slerp(const Quat &q1, const Quat &q2, const Quat::value_type t)
{
    Quat ret;
    ret.slerp(t, q1, q2);
    return ret;
}

const Quat QuatUtils::slerpNoInvert(const Quat &from, const Quat &to, const Quat::value_type t) 
{
	static const double lowerRange = -1.0 + QUAT_EPSILON;
	static const double upperRange = -lowerRange;

    double scale_from = 1.0 - t;
    // this is a dot product
    double cosomega = from.asVec4() * to.asVec4();

    if(lowerRange < cosomega && cosomega < upperRange)
    {
        double omega = std::acos(cosomega) ;  // 0 <= omega <= Pi (see man acos)
        //double sinomega = std::sin(omega) ;  // this sinomega should always be +ve so
        // could try sinomega=sqrt(1-cosomega*cosomega) to avoid a sin()?
        return ((from * std::sin(scale_from * omega)) + (to * std::sin(t * omega))) / std::sin(omega);
    }

	/* --------------------------------------------------
		If quaternions are close to each other we can skip else clause
        The ends of the vectors are very close
        we can use simple linear interpolation - no need
        to worry about the "spherical" interpolation
        -------------------------------------------------- */

    return (from * scale_from) + (to * t);		
}

//! spherical cubic interpolation
const Quat QuatUtils::squad(const Quat &q1,const Quat &q2,const Quat &a,const Quat &b, const Quat::value_type t)
{
	Quat c(slerpNoInvert(q1, q2, t));
	Quat d(slerpNoInvert(a, b, t));
	return slerpNoInvert(c, d, 2.0 * t * (1.0 - t));
}

const Quat QuatUtils::simpleSquadCall(const Quat &q1,const Quat &q2,const Quat &q3,const Quat &q4, const Quat::value_type t)
{
    return squad(q2, q3, spline(q1, q2, q3), spline(q2, q3, q4), t);
}

//! Shoemake-Bezier interpolation using De Castlejau algorithm
const Quat QuatUtils::bezier(const Quat &q1, const Quat &q2, const Quat &a, const Quat &b, const Quat::value_type t)
{
	// level 1
	Quat q11(slerpNoInvert(q1, a, t));
	Quat q12(slerpNoInvert(a, b, t));
	Quat q13(slerpNoInvert(b, q2, t));
	// level 2 and 3
	return slerpNoInvert(slerpNoInvert(q11, q12, t), slerpNoInvert(q12, q13, t), t);
}

const Quat QuatUtils::simpleBezierCall(const Quat &q1,const Quat &q2,const Quat &q3,const Quat &q4, const Quat::value_type t)
{
    return bezier(q2, q3, spline(q1, q2, q3), spline(q2, q3, q4), t);
}

//! Given 3 quaternions, qn-1,qn and qn+1, calculate a control point to be used in spline interpolation
const Quat QuatUtils::spline(const Quat &qnm1, const Quat &qn, const Quat &qnp1)
{
	//inverse should be done but only unit quaternions are expected so conjugate is more efficient!!
	Quat qni(qn.conj());	
	return qn * exp(( log(qni * qnm1) + log(qni * qnp1)) / -4.0);
}

void QuatUtils::quaterionToEuler(const double x, const double y,
	const double z, const double w, double & roll, double & pitch,
	double & yaw)
{
	roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (std::pow(x, 2.0) + std::pow(y, 2.0)));
	pitch = std::asin(2.0 * (w * y - x * z));
	yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (std::pow(y, 2.0) + std::pow(z, 2.0)));
}

void QuatUtils::eulerToQuaternion(const double roll, const double pitch,
	const double yaw, double & x, double & y, double & z, double & w)
{
	Vec3d halfEuler(roll, pitch, yaw);
	halfEuler /= 2.0;
	double c1 = std::cos(halfEuler.z());
	double s1 = std::sin(halfEuler.z());
	double c2 = std::cos(halfEuler.y());
	double s2 = std::sin(halfEuler.y());
	double c3 = std::cos(halfEuler.x());
	double s3 = std::sin(halfEuler.x());
	double c1c2 = c1*c2;
	double s1s2 = s1*s2;
	double c1s2 = c1*s2;
	double s1c2 = s1*c2;
	w = c1c2*c3 - s1s2*s3;
	x = c1c2*s3 + s1s2*c3;
	y = s1c2*c3 + c1s2*s3;
	z = c1s2*c3 - s1c2*s3;
}