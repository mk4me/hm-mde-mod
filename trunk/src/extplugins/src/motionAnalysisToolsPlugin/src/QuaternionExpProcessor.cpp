#include "QuaternionExpProcessor.h"
#include <QuatUtils/QuatUtils.h>

QuaternionExpProcessor::QuaternionExpProcessor()
{
	inPinA = new JointAnglesInputPin(this);
	outPinA = new JointAnglesOutputPin(this);
	addInputPin(inPinA);
	addOutputPin(outPinA);
}

QuaternionExpProcessor::~QuaternionExpProcessor()
{

}

void QuaternionExpProcessor::process()
{
	auto inQuatData = inPinA->getValue();

	//kopiuje go - przygotowuje sobie porcje danych wyjsciowych
	kinematic::JointAngleChannelPtr outQuatData(new kinematic::JointAngleChannel(inQuatData->getSamplesPerSecond()));
	outQuatData->setName("Exponent channel");
	outQuatData->setTimeBaseUnit(inQuatData->getTimeBaseUnit());
	outQuatData->setValueBaseUnit(inQuatData->getValueBaseUnit());

	//modyfikuje dane ktore b�d� zaraz udostepnia� na wyj�ciu	
	for(kinematic::JointAngleChannel::size_type i = 0; i < inQuatData->size(); ++i){
		outQuatData->addPoint( osg::QuatUtils::exp(inQuatData->value(i)) );
	}

	//zapisuj� zmodyfikowane dane do udost�pniena
	outPinA->setValue(outQuatData);
}

void QuaternionExpProcessor::reset()
{

}