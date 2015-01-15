//
//
//  @ Solution: ImuFilters_FrameWork
//  @ Project : ImuFilters_Lib
//  @ File Name : Utils.h
//  @ Date : 2014-11-18
//  @ Author : Przemys³aw Pruszowski, Kamil Lebek
//
//

#if !defined(_IMUFILTERS_LIB_UTILS_H)
#define _IMUFILTERS_LIB_UTILS_H

// Include Eigen definitions
#include <Eigen/Dense>

//! Global namespace
namespace ImuFilters
{

	//! Internal types and implementations
	namespace Internal
	{

		//! Przemo's Utils
		namespace Utils
		{
			//! 3x2 matrix definition
			typedef Eigen::Matrix<double, 3, 2> Matrix32; 
	
			//! Special matrix type
			Eigen::Matrix3d mkrox(const Eigen::Vector3d &x);
	
			//! Hemisphere align
			Eigen::Quaterniond TestQuat(const Eigen::Quaterniond &Q1, const Eigen::Quaterniond &Q2, const Eigen::Quaterniond &Q);
	
			//! Coefficient-wise negation (not Quaternion negation)
			inline Eigen::Quaterniond QNegCoeff(const Eigen::Quaterniond &Q1) 
			{
				return Eigen::Quaterniond(-Q1.coeffs());
			};

			//! Wahba's algorithm implementation
			Eigen::Quaterniond Wahba(const Matrix32 &C, const Matrix32 &D);

		} // Utils

	} // Internal

} // ImuFilters

#endif // _IMUFILTERS_LIB_UTILS_H