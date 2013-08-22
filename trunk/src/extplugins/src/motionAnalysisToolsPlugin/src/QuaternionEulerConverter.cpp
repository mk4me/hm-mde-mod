#include "QuaternionEulerConverter.h"
#include <QuatUtils/QuatUtils.h>

QuaternionToEulerConverter::QuaternionToEulerConverter()
{
	inPinA = new JointAnglesInputPin(this);
	outPinA = new VectorOutputPin(this);
	addInputPin(inPinA);
	addOutputPin(outPinA);
}

QuaternionToEulerConverter::~QuaternionToEulerConverter()
{

}

void QuaternionToEulerConverter::process()
{
	auto inQuatData = inPinA->getValue();

	//kopiuje go - przygotowuje sobie porcje danych wyjsciowych
	VectorChannelPtr outEulerData(new VectorChannel(inQuatData->getSamplesPerSecond()));
	outEulerData->setName("Quaternions in Roll, Pitch, Yaw form");
	outEulerData->setTimeBaseUnit(inQuatData->getTimeBaseUnit());
	outEulerData->setValueBaseUnit(inQuatData->getValueBaseUnit());

	//modyfikuje dane ktore bêdê zaraz udostepnia³ na wyjœciu	
	for(kinematic::JointAngleChannel::size_type i = 0; i < inQuatData->size(); ++i){
		outEulerData->addPoint( osg::QuatUtils::quaterionToEuler<osg::Quat, osg::Vec3>(inQuatData->value(i)) );
	}

	//zapisujê zmodyfikowane dane do udostêpniena
	outPinA->setValue(outEulerData);
}

void QuaternionToEulerConverter::reset()
{

}


EulerToQuaternionConverter::EulerToQuaternionConverter()
{
	outPinA = new JointAnglesOutputPin(this);
	inPinA = new VectorInputPin(this);
	addInputPin(inPinA);
	addOutputPin(outPinA);
}

EulerToQuaternionConverter::~EulerToQuaternionConverter()
{

}

void EulerToQuaternionConverter::process()
{
	auto inEulerData = inPinA->getValue();

	//kopiuje go - przygotowuje sobie porcje danych wyjsciowych
	kinematic::JointAngleChannelPtr outQuatData(new kinematic::JointAngleChannel(inEulerData->getSamplesPerSecond()));
	outQuatData->setName("Euler angles in Quaternion form");
	outQuatData->setTimeBaseUnit(inEulerData->getTimeBaseUnit());
	outQuatData->setValueBaseUnit(inEulerData->getValueBaseUnit());

	//modyfikuje dane ktore bêdê zaraz udostepnia³ na wyjœciu	
	for(kinematic::JointAngleChannel::size_type i = 0; i < inEulerData->size(); ++i){
		outQuatData->addPoint( osg::QuatUtils::eulerToQuaternion<osg::Quat, osg::Vec3>(inEulerData->value(i)) );
	}

	//zapisujê zmodyfikowane dane do udostêpniena
	outPinA->setValue(outQuatData);
}

void EulerToQuaternionConverter::reset()
{

}