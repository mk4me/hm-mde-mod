#include "EulerNoiseAdderProcessor.h"
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QLabel>
#include <osg/Math>
#include <boost/lexical_cast.hpp>
#include <boost/random.hpp>
#include <QuatUtils/QuatUtils.h>

EulerNoiseAdderProcessor::EulerNoiseAdderProcessor() : noise(0.001)
{
	inPinA = new VectorInputPin(this);
	outPinA = new VectorOutputPin(this);
	addInputPin(inPinA);
	addOutputPin(outPinA);

	widget = new QWidget;
	auto layout = new QHBoxLayout;
	layout->addWidget(new QLabel(tr("Gaussian noise range [rad]:")));
	auto spinBox = new QDoubleSpinBox;
	spinBox->setMaximum(osg::PI);
	spinBox->setMinimum(0.0);
	spinBox->setValue(noise);
	layout->addWidget(spinBox);
	widget->setLayout(layout);

	connect(spinBox, SIGNAL(valueChanged(double)), this, SLOT(valueChanged(double)));
}

void EulerNoiseAdderProcessor::valueChanged(double val)
{
	noise = val;
}

EulerNoiseAdderProcessor::~EulerNoiseAdderProcessor()
{

}

void EulerNoiseAdderProcessor::process()
{

	boost::random::mt19937 rng;
	boost::random::uniform_real_distribution<double> eulerNoiseGenerator( -noise, noise);

	auto inEulerData = inPinA->getValue();

	//kopiuje go - przygotowuje sobie porcje danych wyjsciowych
	VectorChannelPtr outEulerData(new VectorChannel(inEulerData->getSamplesPerSecond()));
	outEulerData->setName("Noisy channel: " + boost::lexical_cast<std::string>(noise));
	outEulerData->setTimeBaseUnit(inEulerData->getTimeBaseUnit());
	outEulerData->setValueBaseUnit(inEulerData->getValueBaseUnit());

	//modyfikuje dane ktore bêdê zaraz udostepnia³ na wyjœciu	
	for(VectorChannel::size_type i = 0; i < inEulerData->size(); ++i){
		//skalowanie

		auto val = inEulerData->value(i) + osg::Vec3(
			eulerNoiseGenerator(rng),
			eulerNoiseGenerator(rng),
			eulerNoiseGenerator(rng));

		osg::QuatUtils::clampEuler(val);

		outEulerData->addPoint( val );
	}

	//zapisujê zmodyfikowane dane do udostêpniena
	outPinA->setValue(outEulerData);
}

void EulerNoiseAdderProcessor::reset()
{

}

QWidget* EulerNoiseAdderProcessor::getConfigurationWidget()
{
	return widget;
}

void EulerNoiseAdderProcessor::refreshConfiguration()
{

}