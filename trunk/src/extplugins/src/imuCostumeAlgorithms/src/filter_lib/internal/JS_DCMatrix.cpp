/*
   Autor: Janusz Slupik
   Last update: 25.20.2012
*/
#include "JS_DCMatrix.h"

//! Global namespace
namespace ImuFilters
{

	//! Internal types and implementations
	namespace Internal
	{

		//! Janusz S³upik's tools
		namespace JS
		{
			DCMatrix::DCMatrix( Quaternion &q )
			{
				double q1k = q.b * q.b;
				double q2k = q.c * q.c;
				double q3k = q.d * q.d;
				double q4k = q.a * q.a;

				operator()(0,0) = q1k - q2k - q3k + q4k;
				operator()(0,1) = 2 * ( q.b * q.c + q.d * q.a );
				operator()(0,2) = 2 * ( q.b * q.d - q.c * q.a );
				operator()(1,0) = 2 * ( q.b * q.c - q.d * q.a );
				operator()(1,1) = - q1k + q2k - q3k + q4k;
				operator()(1,2) = 2 * ( q.c * q.d + q.b * q.a );
				operator()(2,0) = 2 * ( q.b * q.d + q.c * q.a );
				operator()(2,1) = 2 * ( q.c * q.d - q.b * q.a );
				operator()(2,2) = - q1k - q2k + q3k + q4k;

				operator*=( 1/q.Qnorm() );
			}
		} // JS

	} // Internal

} // ImuFilters
