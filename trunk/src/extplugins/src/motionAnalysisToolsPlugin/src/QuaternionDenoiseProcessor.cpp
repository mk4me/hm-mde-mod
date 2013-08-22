#include "QuaternionDenoiseProcessor.h"
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QLabel>
#include <osg/Math>
#include <boost/lexical_cast.hpp>

QuaternionDenoiseProcessor::QuaternionDenoiseProcessor() : threshold(0.001)
{
	inPinA = new JointAnglesInputPin(this);
	outPinA = new JointAnglesOutputPin(this);
	addInputPin(inPinA);
	addOutputPin(outPinA);

	widget = new QWidget;
	auto layout = new QHBoxLayout;
	layout->addWidget(new QLabel(tr("Filtering threshold value [degrees]:")));
	auto spinBox = new QDoubleSpinBox;
	spinBox->setMaximum(180.0 - 0.00001);
	spinBox->setMinimum(0.00001);
	spinBox->setValue(threshold);
	layout->addWidget(spinBox);
	widget->setLayout(layout);

	connect(spinBox, SIGNAL(valueChanged(double)), this, SLOT(valueChanged(double)));
}

void QuaternionDenoiseProcessor::valueChanged(double val)
{
	threshold = val / (2.0 * osg::PI);
}

QuaternionDenoiseProcessor::~QuaternionDenoiseProcessor()
{

}

void QuaternionDenoiseProcessor::process()
{
	auto inQuatData = inPinA->getValue();

	//kopiuje go - przygotowuje sobie porcje danych wyjsciowych
	kinematic::JointAngleChannelPtr outQuatData(new kinematic::JointAngleChannel(inQuatData->getSamplesPerSecond()));
	outQuatData->setName("Filtered channel: " + boost::lexical_cast<std::string>(threshold));
	outQuatData->setTimeBaseUnit(inQuatData->getTimeBaseUnit());
	outQuatData->setValueBaseUnit(inQuatData->getValueBaseUnit());

	//modyfikuje dane ktore bêdê zaraz udostepnia³ na wyjœciu	
	for(kinematic::JointAngleChannel::size_type i = 0; i < inQuatData->size(); ++i){
		//skalowanie

		auto val = inQuatData->value(i);
		auto w = std::acos(val.w()) / osg::PI;

		if( !( (w > threshold) && (w < 1.0 - threshold) ) ){
			outQuatData->addPoint( osg::Quat(0.0, 0.0, 0.0, 1.0) );
		}else{
			outQuatData->addPoint( val );
		}		
	}

	//zapisujê zmodyfikowane dane do udostêpniena
	outPinA->setValue(outQuatData);
}

void QuaternionDenoiseProcessor::reset()
{

}

QWidget* QuaternionDenoiseProcessor::getConfigurationWidget()
{
	return widget;
}

void QuaternionDenoiseProcessor::refreshConfiguration()
{

}