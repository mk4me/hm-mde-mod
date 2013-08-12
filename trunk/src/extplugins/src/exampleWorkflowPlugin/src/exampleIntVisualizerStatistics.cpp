#include "PCH.h"
#include "exampleIntVisualizerStatistics.h"
#include <exampleWorkflowPlugin/exampleIntStatistics.h>
#include <QtGui/QTextEdit>

ExampleIntVisualizerStatistics::StatsSerie::StatsSerie( QTextEdit * widget ) : widget(widget)
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
    ExampleIntStatisticsConstPtr statistics = data->get();
    this->widget->clear();
    QString content;
    content += QString("Mean: %1\n").arg( statistics->getMean());
    content += QString("Second Moment: %1\n").arg( statistics->getSecondMoment());
    content += QString("Kurtosis: %1\n").arg( statistics->getKurtosis());
    content += QString("Max: %1\n").arg( statistics->getMax());
    content += QString("Min: %1\n").arg( statistics->getMin());
    content += QString("Median: %1\n").arg( statistics->getMedian());
    content += QString("Skewness: %1\n").arg( statistics->getSkewness());
    this->widget->setText(content);
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
	QTextEdit* edit = new QTextEdit();
	widget->layout()->addWidget(edit);
	StatsSerie * serie = new StatsSerie(edit);
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
