/*
   Autor: Janusz Slupik
   Last update: 25.20.2012
*/

// Includes
#include <cmath>
#include "JS_Quaternion.h"

// Own defines
#ifndef M_PI
	#define M_PI	3.14159265358979323846
#endif

//! Global namespace
namespace ImuFilters
{

	//! Internal types and implementations
	namespace Internal
	{

		//! Janusz S³upik's tools
		namespace JS
		{
			Quaternion::Quaternion() : a(1.0),b(0.0),c(0.0),d(0.0)
			{}

			Quaternion::Quaternion( double w, double x, double y, double z )
			{
			   a = w;
			   b = x;
			   c = y;
			   d = z;
			}

			double Quaternion::Qnorm( void ) const
			{
			   return sqrt( a*a + b*b + c*c + d*d );
			}

			Quaternion Quaternion::Qinverse( void ) const
			{
			   Quaternion p( a, -b, -c, -d );
			   return ( a*a + b*b + c*c + d*d ) * p;
			}

			Quaternion Quaternion::Qconj( void ) const
			{
			   Quaternion q( a, -b, -c, -d );
			   return q;
			}

			Quaternion Quaternion::Qsqrt( void ) const
			{
			   Quaternion p;
			   p += *this; //1+Q
			   double n = p.Qnorm();
			   if( n!=0 ) p = p*(1/n);
			   else
			   {
				  Quaternion q(0,1,0,0);
				  return q; // any 0+n is correct
			   }
			   return p;
			}

			Quaternion Quaternion::QsqrtI( void ) const
			{
			   Quaternion p;
			   p += *this;
			   p = p.Qconj(); 
			   double n = p.Qnorm();
			   if( n!=0 ) p = p*(1/n);
			   else
			   {
				  Quaternion q(0,-1,0,0);
				  return q; // any 0+n is correct
			   }
			   return p;
			}

			Point3D Quaternion::log( void ) const
			{
			   Point3D p;
			   double aa = a;
			   if( aa > 1 ) aa = 1; 
			   double fi = acos( aa ); //naprawde to fi/2
			   double s = sin( fi );
			   if( s!= 0 )
			   {
				  p.x = (fi*b)/s;
				  p.y = (fi*c)/s;
				  p.z = (fi*d)/s;
			   }
			   return p;
			}

			Quaternion Quaternion::operator+ ( const Quaternion& q ) const
			{
			   Quaternion p( a + q.a, b + q.b, c + q.c, d + q.d );
			   return p;
			}

			Quaternion& Quaternion::operator+= ( const Quaternion& q )
			{
			   a += q.a;
			   b += q.b;
			   c += q.c; 
			   d += q.d; 
			   return *this;
			}

			Quaternion Quaternion::operator- ( const Quaternion& q ) const
			{
			   Quaternion p( a - q.a, b - q.b, c - q.c, d - q.d );
			   return p;
			}

			Quaternion Quaternion::operator- ( void ) const
			{
			   Quaternion p( -a, -b, -c, -d );
			   return p;
			}

			Quaternion& Quaternion::operator-= ( const Quaternion& q )
			{
			   a -= q.a;
			   b -= q.b;
			   c -= q.c; 
			   d -= q.d; 
			   return *this;
			}

			Quaternion Quaternion::operator* ( const Quaternion& q ) const
			{
			   Quaternion p;
			   p.a = a*q.a - b*q.b - c*q.c - d*q.d;
			   p.b = c*q.d - q.c*d + a*q.b + q.a*b;
			   p.c = q.b*d - b*q.d + a*q.c + q.a*c;
			   p.d = b*q.c - q.b*c + a*q.d + q.a*d;
			   return p;
			}

			Quaternion& Quaternion::operator*= ( const Quaternion& q )
			{
			   double ta = a*q.a - b*q.b - c*q.c - d*q.d;
			   double tb = c*q.d - q.c*d + a*q.b + q.a*b;
			   double tc = q.b*d - b*q.d + a*q.c + q.a*c;
			   double td = b*q.c - q.b*c + a*q.d + q.a*d;
			   a = ta; b = tb; c = tc; d = td;
			   return *this;
			}

			Quaternion Quaternion::operator* ( const double& r ) const
			{
			   Quaternion q( r*a, r*b, r*c, r*d );
			   return q;
			}

			Quaternion Quaternion::pow( double x ) const
			{
			   Quaternion p( a, b, c, d );
			   Point3D s = p.log();
			   s.x *= x; s.y *= x; s.z *= x;
			   return s.exp();
			}

			Quaternion operator* ( const double& r, const Quaternion& q )
			{
			   Quaternion p( r*q.a, r*q.b, r*q.c, r*q.d );
			   return p;
			}

			void Quaternion::setFromEuler( double rx, double ry, double rz )
			{
			   rx *= 0.00872664626; //stopnie na radiany przez dwa
			   ry *= 0.00872664626;
			   rz *= 0.00872664626;
			   double c1 = cos( rx );
			   double s1 = sin( rx );
			   double c2 = cos( ry );
			   double s2 = sin( ry );
			   double c3 = cos( rz );
			   double s3 = sin( rz );

			   a = c1*c2*c3 + s1*s2*s3;
  				b = s1*c2*c3 - c1*s2*s3;
				c = c1*s2*c3 + s1*c2*s3;
				d = c1*c2*s3 - s1*s2*c3;
			}

			void Quaternion::setFromEuler( Point3D p )
			{
			   setFromEuler( p.x, p.y, p.z );
			}

			std::ostream& operator<< ( std::ostream& o , const Quaternion& q )
			{
			   o << q.a << " + " << q.b << "i + " << q.c << "j + " << q.d << "k ";
			   return o;
			}

			Point3D Quaternion::toEuler( void ) //only for unit quaternions
			{
			   Point3D angles;

			   double tt = 2*(a*c-b*d); 
			   if (tt < -1) 
			   { // singularity at north pole
				  angles.x = 2 * atan2(b,a);
				  angles.y = -M_PI/2;
				  angles.z = 0;
			   }
			   else if (tt > 1) 
			   { // singularity at south pole
				  angles.x= 2 * atan2(b,a);
				  angles.y = M_PI/2;
				  angles.z  = 0;
			   }
			   else
			   {
				  angles.x = atan2(2*(a*b+c*d) , 1-2*(b*b+c*c) );
				  angles.y = asin( tt );
				  angles.z  = atan2(2*(a*d+c*b) , 1-2*(c*c+d*d) );
			   }

			   angles.x *= 57.295779513082;
			   angles.y *= 57.295779513082;
			   angles.z *= 57.295779513082;

			   return angles;
			}

		} // JS

	} // Internal

} // ImuFilters