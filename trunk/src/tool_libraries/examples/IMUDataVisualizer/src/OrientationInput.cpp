#include "OrientationInput.h"
#include <QuatUtils/QuatUtils.h>
#include <osg/Vec3>

OrientationInput::OrientationInput(QWidget * parent, Qt::WindowFlags f)
	: QDialog(parent, f), orientation_(0.0, 0.0, 0.0, 1.0)
{
	setupUi(this);
}

OrientationInput::~OrientationInput()
{

}

const osg::Quat & OrientationInput::orientation() const
{
	return orientation_;
}

void OrientationInput::quaternionChanged()
{
	//aktualizujemy orientacjê
	orientation_.x() = xDoubleSpinBox->value();
	orientation_.y() = yDoubleSpinBox->value();
	orientation_.z() = zDoubleSpinBox->value();
	orientation_.w() = wDoubleSpinBox->value();
	
	updateQuaternionLimits();
}

void OrientationInput::updateCardanAngles()
{
	osg::Vec3 euler = osg::QuatUtils::quaterionToEuler
		<osg::Quat, osg::Vec3>(orientation_);

	blockSignals(true);

	rollDoubleSpinBox->setValue(osg::RadiansToDegrees(euler.x()));
	pitchDoubleSpinBox->setValue(osg::RadiansToDegrees(euler.y()));
	yawDoubleSpinBox->setValue(osg::RadiansToDegrees(euler.z()));

	blockSignals(false);
}

void OrientationInput::updateQuaternion()
{
	updateQuaternionLimits();

	blockSignals(true);

	xDoubleSpinBox->setValue(orientation_.x());
	yDoubleSpinBox->setValue(orientation_.y());
	zDoubleSpinBox->setValue(orientation_.z());
	wDoubleSpinBox->setValue(orientation_.z());

	blockSignals(false);
}

void OrientationInput::cardanChanged()
{
	orientation_ = osg::QuatUtils::eulerToQuaternion
		<osg::Quat, osg::Vec3>(osg::Vec3(rollDoubleSpinBox->value(),
		pitchDoubleSpinBox->value(),
		yawDoubleSpinBox->value()));

	updateQuaternion();
}

void OrientationInput::updateQuaternionLimits()
{
	const double left = std::sqrt(1.0 - ( std::pow(orientation_.x(), 2.0) +
		std::pow(orientation_.y(), 2.0), + std::pow(orientation_.z(), 2.0) +
		std::pow(orientation_.w(), 2.0) ));

	xDoubleSpinBox->setMaximum(orientation_.x() + left);
	xDoubleSpinBox->setMinimum(orientation_.x() - left);
	yDoubleSpinBox->setMaximum(orientation_.y() + left);
	yDoubleSpinBox->setMinimum(orientation_.y() - left);
	zDoubleSpinBox->setMaximum(orientation_.z() + left);
	zDoubleSpinBox->setMinimum(orientation_.z() - left);
	wDoubleSpinBox->setMaximum(orientation_.w() + left);
	wDoubleSpinBox->setMinimum(orientation_.w() - left);
}