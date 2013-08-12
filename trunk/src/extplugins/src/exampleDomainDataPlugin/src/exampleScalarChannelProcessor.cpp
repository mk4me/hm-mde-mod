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
	ScalarChannelReaderInterfaceConstPtr inScalarData = inPinA->getValue();
	        
	//kopiuje go - przygotowuje sobie porcje danych wyjsciowych
	ScalarChannelPtr outScalarData(new ScalarChannel(inScalarData->getSamplesPerSecond()));
	outScalarData->setName("Result channel");
	outScalarData->setTimeBaseUnit(inScalarData->getTimeBaseUnit());
	outScalarData->setValueBaseUnit(inScalarData->getValueBaseUnit());
	
	//modyfikuje dane ktore b�d� zaraz udostepnia� na wyj�ciu
	//for(auto it = inScalarData->begin(); it != inScalarData->end(); it++){
	for(ScalarChannel::size_type i = 0; i < inScalarData->size(); ++i){
	    //skalowanie
		outScalarData->addPoint( inScalarData->value(i) * scale);
	}
	
	//zapisuj� zmodyfikowane dane do udost�pniena
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
