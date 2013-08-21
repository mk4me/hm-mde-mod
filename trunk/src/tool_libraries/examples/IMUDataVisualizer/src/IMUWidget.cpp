#include "IMUWidget.h"
#include <QuatUtils/QuatUtils.h>

IMUWidget::IMUWidget(ITStick * stick, osg::Switch * root, QWidget * parent, Qt::WindowFlags f)
	: pat_(new osg::PositionAttitudeTransform), root_(root), stick_(stick)
{
	pat_->addChild(stick_->asNode());
	root->addChild(pat_);

	setupUi(this);
}

IMUWidget::~IMUWidget()
{

}

void IMUWidget::setLocalAttitude(const IMU::Vec3 & localAttitude)
{
	localRollLabel->setText(QString::number(localAttitude.x()));
	localPitchLabel->setText(QString::number(localAttitude.y()));
	localYawLabel->setText(QString::number(localAttitude.z()));
}

void IMUWidget::updateConnectionsView()
{
	stick_->setConnectionsColor(Qt::green);
	stick_->setConnectionsRadius(this->cRadiusDoubleSpinBox->value());
}

void IMUWidget::updateSpheresView()
{
	//stick_->setSpheresColor(Qt::green);
	stick_->setSpheresRadius(this->sRadiusDoubleSpinBox->value());
}

void IMUWidget::updateAttitude()
{
	osg::Vec3 attitude;

	attitude.x() = osg::DegreesToRadians(this->roll->value());
	attitude.y() = osg::DegreesToRadians(this->pitch->value());
	attitude.z() = osg::DegreesToRadians(this->yaw->value());

	osg::Quat q = osg::QuatUtils::eulerToQuaternion<osg::Quat,osg::Vec3>(attitude);

	pat_->setAttitude(q);
}

void IMUWidget::updatePosition()
{
	osg::Vec3 position;

	position.x() = this->xPos->value();
	position.y() = this->yPos->value();
	position.z() = this->zPos->value();	

	pat_->setPosition(position);
}

void IMUWidget::switchVisibility(bool visible)
{
	root_->setChildValue(pat_, visible);
}