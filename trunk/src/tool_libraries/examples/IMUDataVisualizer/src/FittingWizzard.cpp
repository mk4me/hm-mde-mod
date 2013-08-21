#include "FittingWizzard.h"
#include <osg/Vec3>
#include <osg/Math>
#include <QuatUtils/QuatUtils.h>

FittingWizard::FittingWizard(unsigned int sizeSrc, unsigned int sizeDest,
	QWidget * parent, Qt::WindowFlags f)
	: QWizard(parent, f), offset_(-1), inputOrientationType_(LOCAL_SENSOR),
	offsetDetectionStep_(0), minimalMatchDistance_(0),
	offsetAutodetection_(true), sizeSrc_(sizeSrc), sizeDest_(sizeDest)
{

	if(sizeDest_ == 0 || sizeSrc_ == 0){
		throw std::runtime_error("One of signals is empty");
	}

	setupUi(this);

	offsetSpinBox->blockSignals(true);
	offsetSpinBox->setMinimum(1 - sizeSrc_);
	offsetSpinBox->setMaximum(sizeDest_ - 1);
	offsetSpinBox->setValue(0);
	offsetSpinBox->blockSignals(false);

	offsetStepSpinBox->blockSignals(true);
	offsetStepSpinBox->setMinimum(1);
	offsetStepSpinBox->setMaximum(sizeDest_ - 1);
	offsetStepSpinBox->setValue(0.02 * offsetStepSpinBox->maximum());
	offsetDetectionStep_ = offsetStepSpinBox->value();
	offsetStepSpinBox->blockSignals(false);

	minFitSizeSpinBox->blockSignals(true);
	minFitSizeSpinBox->setMinimum(1);
	minFitSizeSpinBox->setMaximum(std::min(sizeDest_, sizeSrc_));
	minFitSizeSpinBox->setValue(minFitSizeSpinBox->maximum()-1);
	minimalMatchDistance_ = minFitSizeSpinBox->value();
	minFitSizeSpinBox->blockSignals(false);
}

FittingWizard::~FittingWizard()
{

}

const unsigned int FittingWizard::minimalMatchDistance() const
{
	return minimalMatchDistance_;
}

const int FittingWizard::offset() const
{
	return offset_;
}

const IMU::Quat & FittingWizard::orientation() const
{
	return orientation_;
}

const FittingWizard::InputOrientation FittingWizard::inputOrientationType() const
{
	return inputOrientationType_;
}

void FittingWizard::orientationGroupChanged(int idx)
{
	if(orientationButtonGroup->button(idx) == localSensorRadioButton){
		inputOrientationType_ = LOCAL_SENSOR;
	}else{
		inputOrientationType_ = GLOBAL_SENSOR_FRAME;
	}
}

void FittingWizard::offsetGroupChanged(int idx)
{
	if(offsetButtonGroup->button(idx) == customOffsetRadioButton){		
		offsetAutodetection_ = false;
	}else{
		offsetAutodetection_ = true;
	}

	autoOffsetConfigurationGroupBox->setEnabled(offsetAutodetection_);
	offsetSpinBox->setEnabled(!offsetAutodetection_);
}

void FittingWizard::quaternionChanged()
{
	//aktualizujemy orientacjê
	orientation_.x() = xDoubleSpinBox->value();
	orientation_.y() = yDoubleSpinBox->value();
	orientation_.z() = zDoubleSpinBox->value();
	orientation_.w() = wDoubleSpinBox->value();

	updateQuaternionLimits();
	updateCardanAngles();
}

void FittingWizard::updateCardanAngles()
{
	osg::Vec3 euler = osg::QuatUtils::quaterionToEuler
		<IMU::Quat, osg::Vec3>(orientation_);

	rollDoubleSpinBox->blockSignals(true);
	pitchDoubleSpinBox->blockSignals(true);
	yawDoubleSpinBox->blockSignals(true);


	rollDoubleSpinBox->setValue(osg::RadiansToDegrees(euler.x()));
	pitchDoubleSpinBox->setValue(osg::RadiansToDegrees(euler.y()));
	yawDoubleSpinBox->setValue(osg::RadiansToDegrees(euler.z()));


	rollDoubleSpinBox->blockSignals(false);	
	pitchDoubleSpinBox->blockSignals(false);	
	yawDoubleSpinBox->blockSignals(false);
}

void FittingWizard::updateQuaternion()
{
	xDoubleSpinBox->blockSignals(true);
	yDoubleSpinBox->blockSignals(true);
	zDoubleSpinBox->blockSignals(true);
	wDoubleSpinBox->blockSignals(true);

	updateQuaternionLimits();

	xDoubleSpinBox->setValue(orientation_.x());	
	yDoubleSpinBox->setValue(orientation_.y());	
	zDoubleSpinBox->setValue(orientation_.z());	
	wDoubleSpinBox->setValue(orientation_.z());

	xDoubleSpinBox->blockSignals(false);
	yDoubleSpinBox->blockSignals(false);
	zDoubleSpinBox->blockSignals(false);
	wDoubleSpinBox->blockSignals(false);
}

void FittingWizard::cardanAngleChanged()
{
	osg::Vec3 euler(osg::DegreesToRadians(rollDoubleSpinBox->value()),
		osg::DegreesToRadians(pitchDoubleSpinBox->value()),
		osg::DegreesToRadians(yawDoubleSpinBox->value()));

	orientation_ = osg::QuatUtils::eulerToQuaternion
		<IMU::Quat, osg::Vec3>(euler);

	updateQuaternion();
}

void FittingWizard::updateQuaternionLimits()
{
	const double l = orientation_.norm();

	const double left = std::sqrt(1.0 - l);

	xDoubleSpinBox->setMaximum(orientation_.x() + left);
	xDoubleSpinBox->setMinimum(orientation_.x() - left);
	yDoubleSpinBox->setMaximum(orientation_.y() + left);
	yDoubleSpinBox->setMinimum(orientation_.y() - left);
	zDoubleSpinBox->setMaximum(orientation_.z() + left);
	zDoubleSpinBox->setMinimum(orientation_.z() - left);
	wDoubleSpinBox->setMaximum(orientation_.w() + left);
	wDoubleSpinBox->setMinimum(orientation_.w() - left);
}

void FittingWizard::offsetChanged(int offset)
{
	offset_ = offset;
}

void FittingWizard::offsetStepChanged(int offsetStep)
{
	offsetDetectionStep_ = offsetStep;
}

void FittingWizard::minFitSizeChanged(int minFitSize)
{
	minimalMatchDistance_ = minFitSize;
}

const bool FittingWizard::offsetAutodetection() const
{
	return offsetAutodetection_;
}