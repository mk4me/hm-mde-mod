/*
   Autor: Janusz Slupik
   Last update: 25.20.2012
*/

#if !defined(_IMUFILTERS_LIB_JS_BASEELEMENTS_H)
#define _IMUFILTERS_LIB_JS_BASEELEMENTS_H

// Includes
#include <string>
#include <Eigen/Dense> // TODO: czy tu musi byæ ten Eigen?

//! Global namespace
namespace ImuFilters
{

	//! Internal types and implementations
	namespace Internal
	{

		//! Janusz S³upik's tools
		namespace JS
		{
			class Quaternion;
			class Point
			{
			 public:
			   double x, y;

			   Point(void): x(0.0), y(0.0) {}
			};

			class Point3D
			{
			 public:
			   double x, y, z;
			   Point3D( void ): x(0.0), y(0.0), z(0.0) {}
			   Point3D( double a, double b, double c ): x(a), y(b), z(c) {}
			   Point3D(const Eigen::Vector3d &vec) : x(vec.x()), y(vec.y()), z(vec.z()) {}
			   Point3D operator+ ( const Point3D & ) const;
			   Point3D& operator+= ( const Point3D & );
			   Point3D  operator- ( const Point3D & ) const;
			   Point3D  operator- ( void ) const; 
			   Point3D& operator-= ( const Point3D & );

			   JS::Quaternion exp( );
			   double norm( ) const;
			};

			Point3D operator* ( const double& , const Point3D& );

		} // JS

	} // Internal

} // ImuFilters

#endif // _IMUFILTERS_LIB_JS_BASEELEMENTS_H