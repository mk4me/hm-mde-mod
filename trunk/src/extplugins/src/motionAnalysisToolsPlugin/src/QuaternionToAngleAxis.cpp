#include "QuaternionToAngleAxis.h"

QuaternionToAngleAxisConverter::QuaternionToAngleAxisConverter()
{
	inPinA = new JointAnglesInputPin(this);
	outPinA = new VectorOutputPin(this);
	outPinB = new ScalarOutputPin(this);
	addInputPin(inPinA);
	addOutputPin(outPinA);
	addOutputPin(outPinB);
}

QuaternionToAngleAxisConverter::~QuaternionToAngleAxisConverter()
{

}

void QuaternionToAngleAxisConverter::process()
{
	auto inQuatData = inPinA->getValue();

	//kopiuje go - przygotowuje sobie porcje danych wyjsciowych
	VectorChannelPtr outAxisData(new VectorChannel(inQuatData->getSamplesPerSecond()));
	outAxisData->setName("Quaternion rotation axis");
	outAxisData->setTimeBaseUnit(inQuatData->getTimeBaseUnit());
	outAxisData->setValueBaseUnit("");

	ScalarChannelPtr outAngleData(new ScalarChannel(inQuatData->getSamplesPerSecond()));
	outAngleData->setName("Quaternion rotation angle");
	outAngleData->setTimeBaseUnit(inQuatData->getTimeBaseUnit());
	outAngleData->setValueBaseUnit("rad");

	//modyfikuje dane ktore bêdê zaraz udostepnia³ na wyjœciu	
	for(kinematic::JointAngleChannel::size_type i = 0; i < inQuatData->size(); ++i){
		//skalowanie

		auto val = inQuatData->value(i);
		auto w = 2.0 * std::acos(val.w());
		
		outAxisData->addPoint(val.asVec3());
		outAngleData->addPoint( w );		
	}

	//zapisujê zmodyfikowane dane do udostêpniena
	outPinA->setValue(outAxisData);
	outPinB->setValue(outAngleData);
}

void QuaternionToAngleAxisConverter::reset()
{

}