#include "QuaternionLogProcessor.h"
#include <QuatUtils/QuatUtils.h>

QuaternionLogProcessor::QuaternionLogProcessor()
{
	inPinA = new JointAnglesInputPin(this);
	outPinA = new JointAnglesOutputPin(this);
	addInputPin(inPinA);
	addOutputPin(outPinA);
}

QuaternionLogProcessor::~QuaternionLogProcessor()
{

}

void QuaternionLogProcessor::process()
{
	auto inQuatData = inPinA->getValue();

	//kopiuje go - przygotowuje sobie porcje danych wyjsciowych
	kinematic::JointAngleChannelPtr outQuatData(new kinematic::JointAngleChannel(inQuatData->getSamplesPerSecond()));
	outQuatData->setName("Log channel");
	outQuatData->setTimeBaseUnit(inQuatData->getTimeBaseUnit());
	outQuatData->setValueBaseUnit(inQuatData->getValueBaseUnit());

	//modyfikuje dane ktore b�d� zaraz udostepnia� na wyj�ciu	
	for(kinematic::JointAngleChannel::size_type i = 0; i < inQuatData->size(); ++i){
		outQuatData->addPoint( osg::QuatUtils::log(inQuatData->value(i)) );
	}

	//zapisuj� zmodyfikowane dane do udost�pniena
	outPinA->setValue(outQuatData);
}

void QuaternionLogProcessor::reset()
{

}