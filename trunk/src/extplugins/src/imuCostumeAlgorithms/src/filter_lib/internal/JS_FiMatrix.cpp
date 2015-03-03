/*
   Autor: Janusz Slupik
   Last update: 25.20.2012
*/

// Includes
#include "JS_FiMatrix.h"

//! Global namespace
namespace ImuFilters
{

	//! Internal types and implementations
	namespace Internal
	{
		//! Janusz S³upik's tools
		namespace JS
		{
			FiMatrix::FiMatrix( void )
			{  
			}

			void FiMatrix::setNewOmega( Point3D &omega, double T )
			{
				Quaternion q = ( 0.5 * T * omega ).exp(); // ca³kowanie
				*this <<  q.a,  -q.d, q.c, -q.b,
							q.d,  q.a, -q.b, -q.c,  
						-q.c,  q.b,  q.a, -q.d, 
						q.b, q.c, q.d, q.a;
			}

		} // JS

	} // Internal

} // ImuFilters
