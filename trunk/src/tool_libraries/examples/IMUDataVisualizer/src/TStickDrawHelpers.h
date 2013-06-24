/********************************************************************
    created:  2013/06/22
    created:  22:6:2013   12:51
    filename: TStickDrawHelpers.h
    author:   Mateusz Janiak
    
    purpose:  Klasa narz�dziowa pomagaj�ca w rysowaniu r�d�ki
*********************************************************************/
#ifndef HEADER_GUARD___TSTICKDRAWHELPERS_H__
#define HEADER_GUARD___TSTICKDRAWHELPERS_H__

#include <IMU/Data/VICONDataSample.h>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <QtGui/QColor>

class SimpleTStickDrawHelper
{
public:
	//! Struktura pomocna przy opisuwaniu sceny 3D
	template<class T>
	struct ShapeDescription
	{
		osg::ref_ptr<T> shape;
		osg::ref_ptr<osg::ShapeDrawable> shapeDrawable;		
	};

	typedef ShapeDescription<osg::Sphere> ShpereDescription;

	//! Opis po��czenia - musze rotowa� i pozycjonowa�
	typedef ShapeDescription<osg::Cylinder> CylinderDescription;

public:

	static const CylinderDescription createConnection();

	static const ShpereDescription createSphere();

	//! \param position Pozycja danego punktu r�d�ki
	static void setSpherePosition(ShpereDescription & sphere, const IMU::VICONDataSample::Vec3 & position);

	//! \param color Kolor sfer reprezentuj�cych ko�ce r�d�ki
	static void setSphereColor(ShpereDescription & sphere, const QColor & color);
	//! \param radius Promie� sfer reprezentuj�cych ko�ce r�d�ki
	static void setSphereRadius(ShpereDescription & sphere, const double radius);
	//! \param color Kolor po��cze� w r�d�ce
	static void setConnectionColor(CylinderDescription & conneciton, const QColor & color);
	//! \param radius Promie� po��cze� w r�d�ce
	static void setConnectionRadius(CylinderDescription & conneciton, const double radius);
	//! \param conneciton Po��czenie kt�rego pozycj� aktualizujemy
	//! \param a Pierwsza pozycj� kt�r� ��czy po��czenie
	//! \param b Druga pozycj� kt�r� ��czy po��czenie
	static void updateConnection(CylinderDescription & connection,
		const IMU::VICONDataSample::Vec3 & a,
		const IMU::VICONDataSample::Vec3 & b);


};

#endif	//	HEADER_GUARD___TSTICKDRAWHELPERS_H__
