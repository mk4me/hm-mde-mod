/********************************************************************
    created:  2013/07/22
    created:  22:7:2013   9:35
    filename: Types.h
    author:   Mateusz Janiak
    
    purpose:  Nag³ówek z podstawowymi typami danych dla IMU
*********************************************************************/
#ifndef HEADER_GUARD___TYPES_H__
#define HEADER_GUARD___TYPES_H__

#include <Eigen/Core>
#include <Eigen/Dense>

namespace IMU
{

	//! Typ wektora 3D
	typedef Eigen::Vector3d Vec3;
	//! Typ kwaternionu
	typedef Eigen::Quaterniond Quat;

}

#endif	//	HEADER_GUARD___TYPES_H__
