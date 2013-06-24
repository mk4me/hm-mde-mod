/********************************************************************
    created:  2013/06/22
    created:  22:6:2013   13:13
    filename: PATTStick.h
    author:   Mateusz Janiak
    
    purpose:  Klasa reprezentuj¹ca ró¿d¿kê opisan¹ wymiarami i offsetem
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

	//! \param longLength D³ugoœæ d³u¿szego odcinka ró¿d¿ki
	void setLongLength(const double longLength);
	//! \param shortLength D³ugoœæ krótszego odcinka ró¿d¿ki
	void setShortLength(const double shortLength);
	//! \param offset Przesuniêcie d³u¿szego odcinka odnoœnie œrodka krótszego
	void setOffset(const double offset);

	//! \param position Pozucja punktu przeciêcia osi ró¿dzki
	void setPosition(const IMU::VICONDataSample::Vec3 & position);
	//! \param attitude Orientacja ró¿d¿ki
	void setAttitude(const IMU::VICONDataSample::Vec3 & attitude);

	//! \param color Kolor sfer reprezentuj¹cych koñce ró¿d¿ki
	virtual void setSpheresColor(const QColor & color);
	//! \param radius Promieñ sfer reprezentuj¹cych koñce ró¿d¿ki
	virtual void setSpheresRadius(const double radius);
	//! \param color Kolor po³¹czeñ w ró¿d¿ce
	virtual void setConnectionsColor(const QColor & color);
	//! \param radius Promieñ po³¹czeñ w ró¿d¿ce
	virtual void setConnectionsRadius(const double radius);
	//! \return Ró¿d¿ka jako wêze³ sceny 3D
	virtual osg::Node * asNode();

private:
	//! Inicjalizacja reprezentacji graficznej ró¿d¿ki
	void initialize();

private:
	//! We¿e³ sceny 3D prezentuj¹cy ró¿d¿kê
	osg::ref_ptr<osg::PositionAttitudeTransform> stickNode;
	//! Agregat sfer reprezentuj¹cych koñce ró¿d¿ki
	boost::array<SimpleTStickDrawHelper::ShpereDescription, 3> sferEnds;
	//! Agregat sfer reprezentuj¹cych po³¹czenia
	boost::array<SimpleTStickDrawHelper::CylinderDescription, 2> connections;
};


#endif	//	HEADER_GUARD___PATTSTICK_H__
