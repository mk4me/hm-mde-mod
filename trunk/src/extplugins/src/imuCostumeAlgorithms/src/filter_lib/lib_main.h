//
//
//  @ Solution: ImuFilters_FrameWork
//  @ Project : ImuFilters_Lib
//  @ File Name : lib_main.h
//  @ Date : 2014-11-18
//  @ Author : Kamil Lebek
//
//

#if !defined(_IMUFILTERS_LIB_LIB_MAIN_H)
#define _IMUFILTERS_LIB_LIB_MAIN_H

// Global types
//#include "lib_types.h"
#include "IOrientationFilter.h"

//! Export macro
#if 0
#ifdef IMUFILTERS_LIB_EXPORTS
	#define IMUFILTERS_LIB_API __declspec(dllexport)
#else
	#define IMUFILTERS_LIB_API __declspec(dllimport)
#endif
#endif

//! Calling convention
#define IMUFILTERS_LIB_APIENTRY __stdcall

//! Global namespace
namespace ImuFilters
{
	//! Creates desired Kalman filter - user should destroy object
	/*!
		\param filterType one of the implemented filter types - FT_INSTANTENOUS_KALMAN, FT_AQKF_KALMAN
		\return Returns pointer to the filter interface
	*/
	extern "C" /*IMUFILTERS_LIB_API*/ IOrientationFilter* IMUFILTERS_LIB_APIENTRY createFilter(const IOrientationFilter::FilterType& filterType);

} // ImuFilters


#endif // _IMUFILTERS_LIB_LIB_MAIN_H