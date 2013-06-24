#include "TStickDrawHelpers.h"
#include <osg/Geode>

static const double defaultSphereRadius = 50;
static const double defaultConnectionRadius = 10;



inline const osg::Vec4 qtColorToOSG(const QColor & color)
{
	return osg::Vec4(color.redF(), color.greenF(), color.blueF(), color.alphaF());
}

inline const osg::Vec3 imuVec3ToOSG(const IMU::VICONDataSample::Vec3 & position)
{
	return osg::Vec3(position.x(), position.y(), position.z());
}


const SimpleTStickDrawHelper::CylinderDescription SimpleTStickDrawHelper::createConnection()
{
	CylinderDescription ret;
	ret.shape = new osg::Cylinder(osg::Vec3(0.0,0.0,0.0), defaultConnectionRadius, 10.0);
	ret.shapeDrawable = new osg::ShapeDrawable(ret.shape);	
	return ret;
}

const SimpleTStickDrawHelper::ShpereDescription SimpleTStickDrawHelper::createSphere()
{
	ShpereDescription ret;
	ret.shape = new osg::Sphere(osg::Vec3(0.0,0.0,0.0), defaultSphereRadius);
	ret.shapeDrawable = new osg::ShapeDrawable(ret.shape );	
	return ret;
}

void SimpleTStickDrawHelper::setSpherePosition(ShpereDescription & sphere, const IMU::VICONDataSample::Vec3 & position)
{	
	sphere.shape->setCenter(imuVec3ToOSG(position));	
	sphere.shapeDrawable->dirtyDisplayList();
}

void SimpleTStickDrawHelper::setSphereColor(ShpereDescription & sphere, const QColor & color)
{
	sphere.shapeDrawable->setColor(qtColorToOSG(color));
}

void SimpleTStickDrawHelper::setSphereRadius(ShpereDescription & sphere, const double radius)
{
	sphere.shape->setRadius(radius);
}

void SimpleTStickDrawHelper::setConnectionColor(CylinderDescription & conneciton, const QColor & color)
{
	conneciton.shapeDrawable->setColor(qtColorToOSG(color));
}

void SimpleTStickDrawHelper::setConnectionRadius(CylinderDescription & conneciton, const double radius)
{
	conneciton.shape->setRadius(radius);
}

void SimpleTStickDrawHelper::updateConnection(CylinderDescription & connection,
	const IMU::VICONDataSample::Vec3 & a,
	const IMU::VICONDataSample::Vec3 & b)
{
	osg::Vec3 aa(imuVec3ToOSG(a));
	osg::Vec3 bb(imuVec3ToOSG(b));

	//kierunek
	osg::Vec3 dir = bb - aa;
	//pozycja
	connection.shape->setCenter(aa + dir / 2.0);

	auto l = dir.length();
	//d³ugoœæ
	connection.shape->setHeight(dir.length());

	//orientacja
	osg::Quat rot;

	//normalizujemy kierunek
	dir /= l;

	rot.makeRotate(osg::Vec3(0.0, 0.0, 1.0), dir);

	connection.shape->setRotation(rot);	
	connection.shapeDrawable->dirtyDisplayList();
}