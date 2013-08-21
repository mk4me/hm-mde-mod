/********************************************************************
    created:  2013/05/11
    created:  11:5:2013   16:24
    filename: IIMUOrientationEstimator.h
    author:   Mateusz Janiak
    
    purpose:  Interfejs dla algorytmów estumuj¹cych orientacje na podstawie danych z IMU
*********************************************************************/
#ifndef HEADER_GUARD___IIMUORIENTATIONESTIMATOR_H__
#define HEADER_GUARD___IIMUORIENTATIONESTIMATOR_H__

#include <IMU/Data/IMUDataSample.h>

namespace IMU {

class IIMUOrietnationEstimator
{
public:

	//! Wirtualny destruktor
	virtual ~IIMUOrietnationEstimator() {}

	//! Metoda resetuje stan estymatora
	virtual void reset() = 0;

	//! \param sample Kolejna próbka z IMU
	//! \param orientation [out] Estymowana orientacja
	virtual void estimate(const IMUDataSample & sample, Vec3 & orientation) = 0;
	
	//! \return Nazwa algorytmu
	virtual const std::string name() const = 0;
	//! \return Autor algorytmu (implementacja)
	virtual const std::string author() const = 0;	
};

}

#endif	//	HEADER_GUARD___IIMUORIENTATIONESTIMATOR_H__
