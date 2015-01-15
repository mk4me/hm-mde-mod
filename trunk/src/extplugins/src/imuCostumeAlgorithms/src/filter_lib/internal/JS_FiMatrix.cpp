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
						q.b, q.c, q.d, q.a;//*/
   
				/* *this << q.a, -q.c, q.d, -q.b, 0, 0, 0, 0, 0, 0,
							q.c,  q.a, q.b,  q.d, 0, 0, 0, 0, 0, 0,    
						-q.d, -q.b, q.a,  q.c, 0, 0, 0, 0, 0, 0,
							q.b, -q.d, -q.c, q.a, 0, 0, 0, 0, 0, 0,
						0,      0,    0,    0, 1, 0, 0, 0, 0, 0,
						0,      0,    0,    0, 0, 1, 0, 0, 0, 0,
						0,      0,    0,    0, 0, 0, 1, 0, 0, 0,
						0,      0,    0,    0, 0, 0, 0, 1, 0, 0,
						0,      0,    0,    0, 0, 0, 0, 0, 1, 0,
						0,      0,    0,    0, 0, 0, 0, 0, 0, 1;//*/
			}

		} // JS

	} // Internal

} // ImuFilters