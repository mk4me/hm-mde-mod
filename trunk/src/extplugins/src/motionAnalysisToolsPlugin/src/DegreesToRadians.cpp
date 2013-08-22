#include "DegreesToRadians.h"

EulerDegreesToRadiansConverter::EulerDegreesToRadiansConverter()
{
	inPinA = new VectorInputPin(this);
	outPinA = new VectorOutputPin(this);
	addInputPin(inPinA);
	addOutputPin(outPinA);
}

EulerDegreesToRadiansConverter::~EulerDegreesToRadiansConverter()
{

}

void EulerDegreesToRadiansConverter::process()
{
	auto inEulerData = inPinA->getValue();

	//kopiuje go - przygotowuje sobie porcje danych wyjsciowych
	VectorChannelPtr outEulerData(new VectorChannel(inEulerData->getSamplesPerSecond()));
	outEulerData->setName("Channel in radians");
	outEulerData->setTimeBaseUnit(inEulerData->getTimeBaseUnit());
	outEulerData->setValueBaseUnit("rad");

	//modyfikuje dane ktore bêdê zaraz udostepnia³ na wyjœciu	
	for(VectorChannel::size_type i = 0; i < inEulerData->size(); ++i){
		//skalowanie

		auto val = inEulerData->value(i);		
		outEulerData->addPoint(osg::Vec3(osg::DegreesToRadians(val.x()),
			osg::DegreesToRadians(val.y()),
			osg::DegreesToRadians(val.z())));
	}

	//zapisujê zmodyfikowane dane do udostêpniena
	outPinA->setValue(outEulerData);
}

void EulerDegreesToRadiansConverter::reset()
{

}

EulerRadiansToDegreesConverter::EulerRadiansToDegreesConverter()
{
	inPinA = new VectorInputPin(this);
	outPinA = new VectorOutputPin(this);
	addInputPin(inPinA);
	addOutputPin(outPinA);
}

EulerRadiansToDegreesConverter::~EulerRadiansToDegreesConverter()
{

}

void EulerRadiansToDegreesConverter::process()
{
	auto inEulerData = inPinA->getValue();

	//kopiuje go - przygotowuje sobie porcje danych wyjsciowych
	VectorChannelPtr outEulerData(new VectorChannel(inEulerData->getSamplesPerSecond()));
	outEulerData->setName("Channel in radians");
	outEulerData->setTimeBaseUnit(inEulerData->getTimeBaseUnit());
	outEulerData->setValueBaseUnit("deg");

	//modyfikuje dane ktore bêdê zaraz udostepnia³ na wyjœciu	
	for(VectorChannel::size_type i = 0; i < inEulerData->size(); ++i){
		//skalowanie

		auto val = inEulerData->value(i);		
		outEulerData->addPoint(osg::Vec3(osg::RadiansToDegrees(val.x()),
			osg::RadiansToDegrees(val.y()),
			osg::RadiansToDegrees(val.z())));
	}

	//zapisujê zmodyfikowane dane do udostêpniena
	outPinA->setValue(outEulerData);
}

void EulerRadiansToDegreesConverter::reset()
{

}

ScalarDegreesToRadiansConverter::ScalarDegreesToRadiansConverter()
{
	inPinA = new ScalarInputPin(this);
	outPinA = new ScalarOutputPin(this);
	addInputPin(inPinA);
	addOutputPin(outPinA);
}

ScalarDegreesToRadiansConverter::~ScalarDegreesToRadiansConverter()
{

}

void ScalarDegreesToRadiansConverter::process()
{
	auto inScalarData = inPinA->getValue();

	//kopiuje go - przygotowuje sobie porcje danych wyjsciowych
	ScalarChannelPtr outScalarData(new ScalarChannel(inScalarData->getSamplesPerSecond()));
	outScalarData->setName("Values in radians");
	outScalarData->setTimeBaseUnit(inScalarData->getTimeBaseUnit());
	outScalarData->setValueBaseUnit("rad");

	//modyfikuje dane ktore bêdê zaraz udostepnia³ na wyjœciu	
	for(ScalarChannel::size_type i = 0; i < inScalarData->size(); ++i){
		//skalowanie		
		outScalarData->addPoint(osg::DegreesToRadians(inScalarData->value(i)));
	}

	//zapisujê zmodyfikowane dane do udostêpniena
	outPinA->setValue(outScalarData);
}

void ScalarDegreesToRadiansConverter::reset()
{

}

ScalarRadiansToDegreesConverter::ScalarRadiansToDegreesConverter()
{
	inPinA = new ScalarInputPin(this);
	outPinA = new ScalarOutputPin(this);
	addInputPin(inPinA);
	addOutputPin(outPinA);
}

ScalarRadiansToDegreesConverter::~ScalarRadiansToDegreesConverter()
{

}

void ScalarRadiansToDegreesConverter::process()
{
	auto inScalarData = inPinA->getValue();

	//kopiuje go - przygotowuje sobie porcje danych wyjsciowych
	ScalarChannelPtr outScalarData(new ScalarChannel(inScalarData->getSamplesPerSecond()));
	outScalarData->setName("Values in radians");
	outScalarData->setTimeBaseUnit(inScalarData->getTimeBaseUnit());
	outScalarData->setValueBaseUnit("deg");

	//modyfikuje dane ktore bêdê zaraz udostepnia³ na wyjœciu	
	for(ScalarChannel::size_type i = 0; i < inScalarData->size(); ++i){
		//skalowanie		
		outScalarData->addPoint(osg::RadiansToDegrees(inScalarData->value(i)));
	}

	//zapisujê zmodyfikowane dane do udostêpniena
	outPinA->setValue(outScalarData);
}

void ScalarRadiansToDegreesConverter::reset()
{

}