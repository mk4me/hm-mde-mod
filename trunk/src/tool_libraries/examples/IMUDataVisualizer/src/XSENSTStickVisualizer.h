/********************************************************************
    created:  2013/06/22
    created:  22:6:2013   13:35
    filename: XSENSTStickVisualizer.h
    author:   Mateusz Janiak
    
    purpose:  Klasa realizuj¹ca rysowanie rózd¿ki w oparciu o dane XSENS
*********************************************************************/
#ifndef HEADER_GUARD___XSENSTSTICKVISUALIZER_H__
#define HEADER_GUARD___XSENSTSTICKVISUALIZER_H__

#include "ITStick.h"
#include <IMU/Data/XSENSDataSample.h>

class XSENSTStickVisualizer
{
public:
	XSENSTStickVisualizer(const double shortLength, const double longLength,
		const double offset = 0);

	~XSENSTStickVisualizer();

	void setPosition(const IMU::XSENSDataSample::Vec3 & position);

	ITStick * stickStyle();


};

#endif	//	HEADER_GUARD___XSENSTSTICKVISUALIZER_H__
