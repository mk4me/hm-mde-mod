/*
   Autor: Janusz Slupik
   Last update: 25.20.2012
*/

#if !defined(_IMUFILTERS_LIB_JS_STATEVECTOR_H)
#define _IMUFILTERS_LIB_JS_STATEVECTOR_H

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
			class StateVector : public Eigen::Matrix< double, 4, 1 >
			{
			public:
				StateVector(void);
			
				void putQ( const Quaternion & );
				void putQ( const Eigen::Quaterniond & );
				Quaternion getQ( void ) const;

				StateVector & operator=( const Eigen::Matrix<double, 4 ,1 > & );
			};

		} // JS
		
	} // Internal

} // ImuFilters

#endif // _IMUFILTERS_LIB_JS_STATEVECTOR_H