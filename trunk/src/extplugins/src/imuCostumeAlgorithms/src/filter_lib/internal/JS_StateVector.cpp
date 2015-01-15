/*
   Autor: Janusz Slupik
   Last update: 25.20.2012
*/

// Includes
#include "JS_StateVector.h"

//! Global namespace
namespace ImuFilters
{

	//! Internal types and implementations
	namespace Internal
	{

		//! Janusz S³upik's tools
		namespace JS
		{
			StateVector::StateVector( void )
			{
			   *this << 0, 0, 0, 0;
			}

			void StateVector::putQ( const Eigen::Quaterniond & q)
			{
				operator()(0,0) = q.x();
				operator()(1,0) = q.y();
				operator()(2,0) = q.z();
				operator()(3,0) = q.w();
			}
			void StateVector::putQ( const Quaternion &q )
			{
				operator()(0,0) = q.b;
				operator()(1,0) = q.c;
				operator()(2,0) = q.d;
				operator()(3,0) = q.a;
			}

			Quaternion StateVector::getQ( void ) const
			{
				Quaternion q;
				q.b = operator()(0,0);
				q.c = operator()(1,0);
				q.d = operator()(2,0);
				q.a = operator()(3,0);
				return q;
			}

			StateVector & StateVector::operator=( const Eigen::Matrix<double, 4 ,1 > & from )
			{
				*this << from(0,0), from(1,0), from(2,0), from(3,0);
           
				return *this;
			}

		} // JS

	} // Internal

} // ImuFilters