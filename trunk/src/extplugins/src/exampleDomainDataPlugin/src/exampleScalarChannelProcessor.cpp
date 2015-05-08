#include "exampleScalarChannelProcessor.h"
#include "exampleScalarConfigurationWidget.h"

ExampleScalarChannelProcessor::ExampleScalarChannelProcessor(double scale) : scale(scale)
{
	inPinA = new ScalarInputPin(this);
	outPinA = new ScalarOutputPin(this);
	addInputPin(inPinA);
	addOutputPin(outPinA);

}

void ExampleScalarChannelProcessor::process()
{
	c3dlib::ScalarChannelReaderInterfaceConstPtr inScalarData = inPinA->getValue();
	        
	//kopiuje go - przygotowuje sobie porcje danych wyjsciowych
	c3dlib::ScalarChannelPtr outScalarData(new c3dlib::ScalarChannel(inScalarData->getSamplesPerSecond()));
	outScalarData->setName("Result channel");
	outScalarData->setTimeBaseUnit(inScalarData->getTimeBaseUnit());
	outScalarData->setValueBaseUnit(inScalarData->getValueBaseUnit());
	
	//modyfikuje dane ktore bêdê zaraz udostepnia³ na wyjœciu
	//for(auto it = inScalarData->begin(); it != inScalarData->end(); it++){
	for (c3dlib::ScalarChannel::size_type i = 0; i < inScalarData->size(); ++i){
	    //skalowanie
		outScalarData->addPoint( inScalarData->value(i) * scale);
	}
	
	//zapisujê zmodyfikowane dane do udostêpniena
	outPinA->setValue(outScalarData);
}

void ExampleScalarChannelProcessor::reset()
{
	
}

QWidget* ExampleScalarChannelProcessor::getConfigurationWidget()
{
	return new ExampleScalarConfigurationWidget(this);
}

void ExampleScalarChannelProcessor::refreshConfiguration()
{
}

void ExampleScalarChannelProcessor::setScale( double scale )
{
	this->scale = scale;
}

double ExampleScalarChannelProcessor::getScale() const
{
	return scale;
}
