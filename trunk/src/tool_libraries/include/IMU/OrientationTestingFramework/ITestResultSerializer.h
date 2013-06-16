/********************************************************************
    created:  2013/06/05
    created:  5:6:2013   19:21
    filename: ITestResultSerializer.h
    author:   Mateusz Janiak
    
    purpose:  Interfej klas realizuj¹cych zapis wyników testów
*********************************************************************/
#ifndef HEADER_GUARD___ITESTRESULTSERIALIZER_H__
#define HEADER_GUARD___ITESTRESULTSERIALIZER_H__

#include <IMU/OrientationTestingFramework/TestingFramework.h>

namespace IMU {

class ITestResultSerializer
{
public:
	//! Destruktor wirtualny
	virtual ~ITestResultSerializer() {}
	//! \param results Wyniki testu estymatorów orientacji dla IMU
	virtual void serialize(const TestingFramework::EstimationResults & results,
		const TestingFramework::OrientationData & orientation) = 0;
	//! \param results [out] Wyniki testu estymatorów orientacji dla IMU
	virtual void deserialize(TestingFramework::EstimationResults & results) = 0;
};

}

#endif	//	HEADER_GUARD___ITESTRESULTSERIALIZER_H__
