/*
   Autor: Janusz Slupik
   Last update: 25.20.2012
*/

#if !defined(_IMUFILTERS_LIB_JS_DCMATRIX_H)
#define _IMUFILTERS_LIB_JS_DCMATRIX_H

// Includes
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
			class DCMatrix : public Eigen::Matrix<double, 3, 3>
			{
			public:
				DCMatrix( Quaternion & );

			};

		} // JS

	} // Internal

} // ImuFilters

#endif // _IMUFILTERS_LIB_JS_DCMATRIX_H