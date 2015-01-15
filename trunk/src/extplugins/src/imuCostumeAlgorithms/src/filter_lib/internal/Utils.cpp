//
//
//  @ Solution: ImuFilters_FrameWork
//  @ Project : ImuFilters_Lib
//  @ File Name : Utils.cpp
//  @ Date : 2014-11-18
//  @ Author : Przemys³aw Pruszowski, Kamil Lebek
//
//

#include "Utils.h"

//! Global namespace
namespace ImuFilters
{

	//! Internal types and implementations
	namespace Internal
	{

		//! Przemo's Utils
		namespace Utils
		{
			//! Special matrix type
			Eigen::Matrix3d mkrox(const Eigen::Vector3d &x)
			{
				Eigen::Matrix3d Y;

				Y << 0, -x(2), x(1),
					x(2), 0 ,-x(0),
					-x(1), x(0), 0;

				return Y;
			}
			
			//! Hemisphere align
			Eigen::Quaterniond TestQuat(const Eigen::Quaterniond &Q1, const Eigen::Quaterniond &Q2, const Eigen::Quaterniond &Q)
			{
				Eigen::Quaterniond Qans;

				if (Q1.dot(Q) < 0.0)
					Qans = Q2;
				if ( Q2.dot(Q) < 0.0)
					Qans = Q1;

				return Qans;
			}

			//! Wahba's algorithm implementation
			Eigen::Quaterniond Wahba(const Matrix32 &C,const Matrix32 &D)
			{
				// Local vars
				Eigen::MatrixXd BB(4 * 2, 4);
				Eigen::Vector3d cc;
				Eigen::Vector3d dd;
				Eigen::Vector3d d_c, c_d;
				
				// Loop through...
				for(int i = 0 ; i < 2; ++i)
				{
					cc = C.col(i);
					dd = D.col(i);
					Eigen::Matrix3d BX = Utils::mkrox(cc + dd);
					d_c = dd - cc;
					c_d = cc - dd;
					Eigen::Matrix4d B;
					B(0, 0) = 0;
					B.block(0, 1, 1, 3) = c_d.transpose();
					B.block(1, 1,3 , 3) = BX;
					B.block(1, 0, 3, 1) = d_c;
					BB.block(4 * i, 0, 4, 4) = B;
				}
				
				// Svd
				Eigen::JacobiSVD<Eigen::MatrixXd> svd(BB, Eigen::ComputeThinV);
				
				Eigen::Matrix4d V;
				V = svd.matrixV();
				
				// Make data eigen compatible
				Eigen::Quaterniond Q(V(0, 3), V(1, 3), V(2, 3), V(3, 3));
				//Q.coeffs() << V(1,3),V(2,3),V(3,3),V(0,3); tam tak tu tak, eigen...

				// Return
				return Q;
			}

		} // Utils

	} // Internal

} // ImuFilters