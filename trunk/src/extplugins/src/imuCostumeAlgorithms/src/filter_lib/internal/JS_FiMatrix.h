/*
   Autor: Janusz Slupik
   Last update: 25.20.2012
*/

#if !defined(_IMUFILTERS_LIB_JS_FIMATRIX_H)
#define _IMUFILTERS_LIB_JS_FIMATRIX_H

// Includes
#include <Eigen/Dense>
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
			class FiMatrix : public Eigen::Matrix<double,4,4>
			{
			public:
				FiMatrix( void ); 
				void setNewOmega( Point3D &, double = 0.01  );// wektor prêdkoœci k¹towych, T - system sampling interval
			};

		} // JS

	} // Internal

} // ImuFilters

#endif // _IMUFILTERS_LIB_JS_FIMATRIX_H