/*
   Autor: Janusz Slupik
   Last update: 25.20.2012
*/

#if !defined(_IMUFILTERS_LIB_JS_QUATERNION_H)
#define _IMUFILTERS_LIB_JS_QUATERNION_H

// Includes
#include <iostream>
#include "JS_BaseElements.h"

//! Global namespace
namespace ImuFilters
{

	//! Internal types and implementations
	namespace Internal
	{

		//! Janusz S³upik's tools
		namespace JS
		{
			class Quaternion
			{
				public:
				double a,b,c,d;  //b c d to wektor a to 
				Quaternion();
				Quaternion( double, double, double, double );

				double Qnorm( void ) const;
				Quaternion Qinverse( void ) const;
				Quaternion Qconj( void ) const; //conjugated
				Quaternion Qsqrt( void ) const; //equivalent to q^{0.5}
				Quaternion QsqrtI( void ) const; //equivalent to q^{-0.5}
				Point3D log( void ) const;
   
				void setFromEuler( double rx, double ry, double rz ); //degrees in arguments
				void setFromEuler( JS::Point3D p);
				Point3D toEuler( void ); //only for unit quaternions

				Quaternion operator+ ( const Quaternion& ) const;
				Quaternion& operator+= ( const Quaternion& );
				Quaternion operator- ( const Quaternion& ) const;
				Quaternion operator- ( void ) const; 
				Quaternion& operator-= ( const Quaternion& );
				Quaternion operator* ( const Quaternion& ) const;
				Quaternion& operator*= ( const Quaternion& );
				Quaternion operator* ( const double& ) const;
				Quaternion pow( double x ) const;

				friend Quaternion operator* ( const double& , const Quaternion& );
				friend std::ostream& operator<< ( std::ostream& , const Quaternion& );

				private:

			};

			Quaternion operator* ( const double& , const Quaternion& );
			std::ostream& operator<< ( std::ostream& , const Quaternion& );

		} // JS

	} // Internal

} // ImuFilters


#endif // _IMUFILTERS_LIB_JS_QUATERNION_H