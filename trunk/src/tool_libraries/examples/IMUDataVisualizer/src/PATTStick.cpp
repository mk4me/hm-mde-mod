#include "PATTStick.h"
#include <QuatUtils/QuatUtils.h>
#include <osg/Geode>

PATTStick::PATTStick(const double shortLength, const double longLength,
	const double offset) : stickNode(new osg::PositionAttitudeTransform)
{
	initialize();

	auto halfShort = shortLength / 2.0;

	IMU::VICONDataSample::Vec3 posA(-halfShort, 0.0, 0.0);
	IMU::VICONDataSample::Vec3 posB(halfShort, 0.0, 0.0);
	IMU::VICONDataSample::Vec3 posC(offset, -longLength, 0.0);

	SimpleTStickDrawHelper::setSpherePosition(sferEnds[0], posA);
	SimpleTStickDrawHelper::setSpherePosition(sferEnds[1], posB);
	SimpleTStickDrawHelper::setSpherePosition(sferEnds[2], posC);

	SimpleTStickDrawHelper::updateConnection(connections[0], posA, posB);
	SimpleTStickDrawHelper::updateConnection(connections[1], IMU::VICONDataSample::Vec3(offset, 0.0, 0.0), posC);
}

PATTStick::~PATTStick()
{

}

void PATTStick::setLongLength(const double longLength)
{
	if(longLength <= 0.0){
		throw std::invalid_argument("Wrong length value");
	}

	auto pos = sferEnds[2].shape->getCenter();
	pos.y() = -longLength;

	IMU::VICONDataSample::Vec3 posC(pos.x(), pos.y(), pos.z());

	SimpleTStickDrawHelper::setSpherePosition(sferEnds[2], posC);
	SimpleTStickDrawHelper::updateConnection(connections[1], IMU::VICONDataSample::Vec3(posC.x(), 0.0, 0.0), posC);
}

void PATTStick::setShortLength(const double shortLength)
{
	if(shortLength <= 0.0){
		throw std::invalid_argument("Wrong length value");
	}

	auto halfShort = shortLength / 2.0;

	IMU::VICONDataSample::Vec3 posA(-halfShort, 0.0, 0.0);
	IMU::VICONDataSample::Vec3 posB(halfShort, 0.0, 0.0);	

	SimpleTStickDrawHelper::setSpherePosition(sferEnds[0], posA);
	SimpleTStickDrawHelper::setSpherePosition(sferEnds[1], posB);

	SimpleTStickDrawHelper::updateConnection(connections[0], posA, posB);
}

void PATTStick::setOffset(const double offset)
{
	if(std::fabs(offset) > std::fabs(sferEnds[0].shape->getCenter().x())){
		throw std::invalid_argument("Wrong offset value");
	}

	auto pos = sferEnds[2].shape->getCenter();
	pos.x() = offset;

	IMU::VICONDataSample::Vec3 posC(pos.x(), pos.y(), pos.z());

	SimpleTStickDrawHelper::setSpherePosition(sferEnds[2], posC);
	SimpleTStickDrawHelper::updateConnection(connections[1], IMU::VICONDataSample::Vec3(offset, 0.0, 0.0), posC);
}

void PATTStick::setPosition(const IMU::VICONDataSample::Vec3 & position)
{
	stickNode->setPosition(osg::Vec3(position.x(), position.y(), position.z()));
}

void PATTStick::setAttitude(const IMU::VICONDataSample::Vec3 & attitude)
{
	osg::Quat rot;
	osg::QuatUtils::eulerToQuaternion(attitude.x(), attitude.y(), attitude.z(),
		rot.x(), rot.y(), rot.z(), rot.w());

	stickNode->setAttitude(rot);
}

void PATTStick::setSpheresColor(const QColor & color)
{
	SimpleTStickDrawHelper::setSphereColor(sferEnds[0], color);
	SimpleTStickDrawHelper::setSphereColor(sferEnds[1], color);
	SimpleTStickDrawHelper::setSphereColor(sferEnds[2], color);	
}

void PATTStick::setSpheresRadius(const double radius)
{
	SimpleTStickDrawHelper::setSphereRadius(sferEnds[0], radius);
	SimpleTStickDrawHelper::setSphereRadius(sferEnds[1], radius);
	SimpleTStickDrawHelper::setSphereRadius(sferEnds[2], radius);	
}

void PATTStick::setConnectionsColor(const QColor & color)
{
	SimpleTStickDrawHelper::setConnectionColor(connections[0], color);
	SimpleTStickDrawHelper::setConnectionColor(connections[1], color);	
}

void PATTStick::setConnectionsRadius(const double radius)
{
	SimpleTStickDrawHelper::setConnectionRadius(connections[0], radius);
	SimpleTStickDrawHelper::setConnectionRadius(connections[1], radius);	
}

osg::Node * PATTStick::asNode()
{
	return stickNode;
}

void PATTStick::initialize()
{
	sferEnds[0] = SimpleTStickDrawHelper::createSphere();
	sferEnds[1] = SimpleTStickDrawHelper::createSphere();
	sferEnds[2] = SimpleTStickDrawHelper::createSphere();	

	connections[0] = SimpleTStickDrawHelper::createConnection();
	connections[1] = SimpleTStickDrawHelper::createConnection();
	
	osg::Geode * geode = new osg::Geode;	

	geode->addDrawable(sferEnds[0].shapeDrawable);
	geode->addDrawable(sferEnds[1].shapeDrawable);
	geode->addDrawable(sferEnds[2].shapeDrawable);
	geode->addDrawable(connections[0].shapeDrawable);
	geode->addDrawable(connections[1].shapeDrawable);

	stickNode->addChild(geode);

}