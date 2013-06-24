/********************************************************************
    created:  2013/06/22
    created:  22:6:2013   13:13
    filename: PATTStick.h
    author:   Mateusz Janiak
    
    purpose:  Klasa reprezentuj�ca r�d�k� opisan� wymiarami i offsetem
*********************************************************************/
#ifndef HEADER_GUARD___PATTSTICK_H__
#define HEADER_GUARD___PATTSTICK_H__

#include "ITStick.h"
#include <IMU/Data/VICONDataSample.h>
#include <osg/PositionAttitudeTransform>
#include "TStickDrawHelpers.h"
#include <boost/array.hpp>

class PATTStick : public ITStick
{
public:
	//! Konstruktor
	PATTStick(const double shortLength, const double longLength,
		const double offset = 0);

	virtual ~PATTStick();

	//! \param longLength D�ugo�� d�u�szego odcinka r�d�ki
	void setLongLength(const double longLength);
	//! \param shortLength D�ugo�� kr�tszego odcinka r�d�ki
	void setShortLength(const double shortLength);
	//! \param offset Przesuni�cie d�u�szego odcinka odno�nie �rodka kr�tszego
	void setOffset(const double offset);

	//! \param position Pozucja punktu przeci�cia osi r�dzki
	void setPosition(const IMU::VICONDataSample::Vec3 & position);
	//! \param attitude Orientacja r�d�ki
	void setAttitude(const IMU::VICONDataSample::Vec3 & attitude);

	//! \param color Kolor sfer reprezentuj�cych ko�ce r�d�ki
	virtual void setSpheresColor(const QColor & color);
	//! \param radius Promie� sfer reprezentuj�cych ko�ce r�d�ki
	virtual void setSpheresRadius(const double radius);
	//! \param color Kolor po��cze� w r�d�ce
	virtual void setConnectionsColor(const QColor & color);
	//! \param radius Promie� po��cze� w r�d�ce
	virtual void setConnectionsRadius(const double radius);
	//! \return R�d�ka jako w�ze� sceny 3D
	virtual osg::Node * asNode();

private:
	//! Inicjalizacja reprezentacji graficznej r�d�ki
	void initialize();

private:
	//! We�e� sceny 3D prezentuj�cy r�d�k�
	osg::ref_ptr<osg::PositionAttitudeTransform> stickNode;
	//! Agregat sfer reprezentuj�cych ko�ce r�d�ki
	boost::array<SimpleTStickDrawHelper::ShpereDescription, 3> sferEnds;
	//! Agregat sfer reprezentuj�cych po��czenia
	boost::array<SimpleTStickDrawHelper::CylinderDescription, 2> connections;
};


#endif	//	HEADER_GUARD___PATTSTICK_H__
