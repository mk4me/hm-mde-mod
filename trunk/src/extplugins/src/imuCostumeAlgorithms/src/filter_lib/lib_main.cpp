//
//
//  @ Solution: ImuFilters_FrameWork
//  @ Project : ImuFilters_Lib
//  @ File Name : lib_main.cpp
//  @ Date : 2014-11-18
//  @ Author : Kamil Lebek
//
//

#include "lib_main.h"

// Include implementations
#include "internal\CInstFilter.h"
#include "internal\CAQKfFilter.h"

// Memory leaks
#ifdef _DEBUG
	#include <vld.h>
#endif

namespace ImuFilters
{
	//! Creates desired Kalman filter - user should destroy object
	extern "C" /*IMUFILTERS_LIB_API*/ IOrientationFilter* IMUFILTERS_LIB_APIENTRY createFilter(const IOrientationFilter::FilterType& filterType)
	{
		// Select interface
		switch (filterType)
		{
			// Instantenous filter
			case IOrientationFilter::FT_INSTANTENOUS_KALMAN:
				return Internal::CInstFilter::create();
		
			// AQKF filter
			case IOrientationFilter::FT_AQKF_KALMAN:
				return Internal::CAQKfFilter::create();

			default:
				return NULL;
		}
	}
	
}