/*
   Autor: Janusz Slupik
   Last update: 25.20.2012
*/

// Includes
#include <cmath>
#include "JS_BaseElements.h"
#include "JS_Quaternion.h"

//! Global namespace
namespace ImuFilters
{

	//! Internal types and implementations
	namespace Internal
	{

		//! Janusz S³upik's tools
		namespace JS
		{

			Quaternion Point3D::exp( void )
			{
			   Quaternion q;
			   double fi = sqrt( x*x + y*y + z*z );
			   q.a = cos( fi );
			   double s = sin( fi );
			   if( fi != 0 )
			   {
				  q.b = (s*x)/fi;
				  q.c = (s*y)/fi;
				  q.d = (s*z)/fi;
			   }
			   return q;
			}

			double Point3D::norm( void ) const
			{
			   return sqrt( x * x + y * y + z * z );
			}

			Point3D Point3D::operator+ ( const Point3D &a ) const
			{
			   Point3D n;
			   n.x = x + a.x;
			   n.y = y + a.y;
			   n.z = z + a.z;
			   return n;
			}

			Point3D& Point3D::operator+= ( const Point3D &a )
			{
			   x += a.x;
			   y += a.y;
			   z += a.z;
			   return *this;
			}

			Point3D  Point3D::operator- ( const Point3D &a ) const
			{
			   Point3D n;
			   n.x = x - a.x;
			   n.y = y - a.y;
			   n.z = z - a.z;
			   return n;
			}

			Point3D  Point3D::operator- ( void ) const
			{
			   Point3D n;
			   n.x = (- x);
			   n.y = (- y);
			   n.z = (- z);
			   return n;
			}

			Point3D& Point3D::operator-= ( const Point3D &a )
			{
			   x -= a.x;
			   y -= a.y;
			   z -= a.z;
			   return *this;
			}

			Point3D operator* ( const double& r , const Point3D& a )
			{
			   Point3D n;
			   n.x = r*a.x;
			   n.y = r*a.y;
			   n.z = r*a.z;
			   return n;
			}

		} // JS

	} // Internal

} // ImuFilters