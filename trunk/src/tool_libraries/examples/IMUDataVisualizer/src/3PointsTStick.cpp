#include "3PointsTStick.h"
#include <osg/Geode>
#include <Eigen/Eigen>
#include <osgManipulator/TranslateAxisDragger>

_3PointsTStick::_3PointsTStick()
{
	initialize();

	StickPositionType sp;
	sp[0] = IMU::Vec3(50.0, 0.0, 0.0);
	sp[1] = IMU::Vec3(-50.0, 0.0, 0.0);
	sp[2] = IMU::Vec3(0.0, -200.0, 0.0);

	setPosition(sp);
}

_3PointsTStick::_3PointsTStick(const StickPositionType & initialPosition)
{
	initialize();
	setPosition(initialPosition);
}

_3PointsTStick::~_3PointsTStick()
{

}

void _3PointsTStick::initialize()
{
	sferEnds[0] = SimpleTStickDrawHelper::createSphere();
	sferEnds[1] = SimpleTStickDrawHelper::createSphere();
	sferEnds[2] = SimpleTStickDrawHelper::createSphere();

	connections[0] = SimpleTStickDrawHelper::createConnection();
	connections[1] = SimpleTStickDrawHelper::createConnection();

	stickGroup = new osg::Group;	

	osg::Geode * geode = new osg::Geode;	

	geode->addDrawable(sferEnds[0].shapeDrawable);
	geode->addDrawable(sferEnds[1].shapeDrawable);
	geode->addDrawable(sferEnds[2].shapeDrawable);
	geode->addDrawable(connections[0].shapeDrawable);
	geode->addDrawable(connections[1].shapeDrawable);

	stickGroup->addChild(geode);

	auto axis = new osgManipulator::TranslateAxisDragger;		

	//domyœlna geometria osi
	axis->setupDefaultGeometry();
	axis->setMatrix(osg::Matrix::scale(100.0, 100.0, 100.0));
	axis->setReferenceFrame(osg::Transform::RELATIVE_RF);
	stickGroup->addChild(axis);

}

void _3PointsTStick::setPosition(const StickPositionType & position)
{
	SimpleTStickDrawHelper::setSpherePosition(sferEnds[0], position[0]);
	SimpleTStickDrawHelper::setSpherePosition(sferEnds[1], position[1]);
	SimpleTStickDrawHelper::setSpherePosition(sferEnds[2], position[2]);	
	
	IMU::Vec3 ba = position[1] - position[0];
	IMU::Vec3 ca = position[2] - position[0];

	const double t = ba.dot(ca) / ba.dot(ba);	
	
	SimpleTStickDrawHelper::updateConnection(connections[0], position[0], position[1]);
	SimpleTStickDrawHelper::updateConnection(connections[1], IMU::Vec3(position[0] + ba * t), position[2]);
}

void _3PointsTStick::setSpheresColor(const QColor & color)
{
	SimpleTStickDrawHelper::setSphereColor(sferEnds[0], color);
	SimpleTStickDrawHelper::setSphereColor(sferEnds[1], color);
	SimpleTStickDrawHelper::setSphereColor(sferEnds[2], color);	
}

void _3PointsTStick::setSpheresRadius(const double radius)
{
	SimpleTStickDrawHelper::setSphereRadius(sferEnds[0], radius);
	SimpleTStickDrawHelper::setSphereRadius(sferEnds[1], radius);
	SimpleTStickDrawHelper::setSphereRadius(sferEnds[2], radius);	
}

void _3PointsTStick::setConnectionsColor(const QColor & color)
{
	SimpleTStickDrawHelper::setConnectionColor(connections[0], color);
	SimpleTStickDrawHelper::setConnectionColor(connections[1], color);	
}

void _3PointsTStick::setConnectionsRadius(const double radius)
{
	SimpleTStickDrawHelper::setConnectionRadius(connections[0], radius);
	SimpleTStickDrawHelper::setConnectionRadius(connections[1], radius);	
}

osg::Node * _3PointsTStick::asNode()
{
	return stickGroup;
}