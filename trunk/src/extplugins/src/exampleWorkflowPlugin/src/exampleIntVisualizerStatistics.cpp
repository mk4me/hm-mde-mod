#include "PCH.h"
#include "exampleIntVisualizerStatistics.h"
#include <exampleWorkflowPlugin/exampleIntStatistics.h>

ExampleIntVisualizerStatistics::StatsSerie::StatsSerie( QLineEdit * widget ) : widget(widget)
{

}

void ExampleIntVisualizerStatistics::StatsSerie::setName( const std::string & name )
{
	this->name = name;
}

const std::string ExampleIntVisualizerStatistics::StatsSerie::getName() const
{
	return name;
}

void ExampleIntVisualizerStatistics::StatsSerie::setData( const core::TypeInfo & requestedDataType, const core::ObjectWrapperConstPtr & data )
{
	this->data = data;
	this->requestedType = requestedDataType;
}

void ExampleIntVisualizerStatistics::StatsSerie::update()
{

}

const core::ObjectWrapperConstPtr & ExampleIntVisualizerStatistics::StatsSerie::getData() const
{
	return data;
}

const core::TypeInfo & ExampleIntVisualizerStatistics::StatsSerie::getRequestedDataType() const
{
	return requestedType;
}

ExampleIntVisualizerStatistics::ExampleIntVisualizerStatistics() :
	activeSerie(nullptr)
{

}

plugin::IVisualizer * ExampleIntVisualizerStatistics::create() const
{
	return new ExampleIntVisualizerStatistics();
}

QWidget* ExampleIntVisualizerStatistics::createWidget()
{
	QAction * shuffleAction = new QAction("Shuffle", nullptr);
	connect(shuffleAction, SIGNAL(triggered()), this, SLOT(shuffle()));
	//auto groupID = actionsGroup->createGroup(tr("Operations"));
	//actionsGroup->addGroupAction(groupID, shuffleAction);
	
	widget = new QWidget();
	widget->addAction(shuffleAction);
	QVBoxLayout * layout = new QVBoxLayout();
	widget->setLayout(layout);
	return widget;
}

QIcon* ExampleIntVisualizerStatistics::createIcon()
{
	return new QIcon();
}

QPixmap ExampleIntVisualizerStatistics::takeScreenshot() const
{
	return QPixmap::grabWidget(widget);
}

void ExampleIntVisualizerStatistics::update( double deltaTime )
{

}

plugin::IVisualizer::ISerie* ExampleIntVisualizerStatistics::createSerie( const core::TypeInfo & requestedType, const core::ObjectWrapperConstPtr & data )
{
	QLineEdit* lineEdit = new QLineEdit();
	widget->layout()->addWidget(lineEdit);
	StatsSerie * serie = new StatsSerie(lineEdit);
	serie->setName("Int statistics");
	serie->setData(requestedType, data);
	
	return serie;
}

plugin::IVisualizer::ISerie* ExampleIntVisualizerStatistics::createSerie( const ISerie* serie )
{
	return nullptr;
}

void ExampleIntVisualizerStatistics::removeSerie( ISerie* serie )
{
	StatsSerie * s = dynamic_cast<StatsSerie*>(serie);
	widget->layout()->removeWidget(s->widget);
	s->widget = nullptr;
}

void ExampleIntVisualizerStatistics::setActiveSerie( ISerie * serie )
{
	activeSerie = serie;
}

const plugin::IVisualizer::ISerie * ExampleIntVisualizerStatistics::getActiveSerie() const
{
	return activeSerie;
}

void ExampleIntVisualizerStatistics::getSupportedTypes( core::TypeInfoList & supportedTypes ) const
{
	supportedTypes.push_back(typeid(ExampleIntStatistics));
}

int ExampleIntVisualizerStatistics::getMaxDataSeries() const
{
	return -1;
}

void ExampleIntVisualizerStatistics::shuffle()
{
	if(widget->layout()->children().size() < 2){
	    QMessageBox msgBox;
	    msgBox.setText("For shuffling minimum 2 data series are required.");
	    msgBox.exec();
	    return;
	}
	
	std::vector<QLayoutItem*> items;
	while(widget->layout()->children().empty() == false){
	    items.push_back(widget->layout()->takeAt(0));
	}
	
	std::random_shuffle(items.begin(), items.end());
	
	for(auto it = items.begin(); it != items.end(); it++){
	    widget->layout()->addItem(*it);
	}

}
