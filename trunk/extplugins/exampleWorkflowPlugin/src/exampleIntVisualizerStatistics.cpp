#include "PCH.h"
#include "exampleIntVisualizerStatistics.h"
#include <core/IActionsGroupManager.h>

const std::string& ExampleIntVisualizerStatistics::getName() const
{
    static const std::string name("ExampleIntVisualizerStatistics");
    return name;
}

core::IVisualizer* ExampleIntVisualizerStatistics::createClone() const
{
    return new ExampleIntVisualizerStatistics();
}

void ExampleIntVisualizerStatistics::getInputInfo(std::vector<core::IInputDescription::InputInfo>& info)
{
    core::IInputDescription::InputInfo input;
    input.name = "STATS";
    input.modify = false;
    input.required = true;
    input.type = typeid(ExampleIntStatistics);

    info.push_back(input);
}

void ExampleIntVisualizerStatistics::update(double deltaTime)
{

}

QWidget* ExampleIntVisualizerStatistics::createWidget(core::IActionsGroupManager * actionsGroup)
{
    QAction * shuffleAction = new QAction("Shuffle", nullptr);
    connect(shuffleAction, SIGNAL(triggered()), this, SLOT(shuffle()));
    auto groupID = actionsGroup->createGroup(tr("Operations"));
	actionsGroup->addGroupAction(groupID, shuffleAction);

    widget = new QWidget();
    QVBoxLayout * layout = new QVBoxLayout();
    widget->setLayout(layout);
    return widget;
}

QIcon* ExampleIntVisualizerStatistics::createIcon()
{
	return nullptr;
}

QPixmap ExampleIntVisualizerStatistics::print() const
{
	return QPixmap::grabWidget(widget);
}

void ExampleIntVisualizerStatistics::setUp(core::IObjectSource* source)
{
    reset();
}

int ExampleIntVisualizerStatistics::getMaxDataSeries() const
{
    return -1;
}

core::IVisualizer::SerieBase* ExampleIntVisualizerStatistics::createSerie(const core::ObjectWrapperConstPtr & data, const std::string & name)
{
    QLineEdit* lineEdit = new QLineEdit();
    widget->layout()->addWidget(lineEdit);
    StatsSerie * serie = new StatsSerie(lineEdit);
    serie->setName(name);
    serie->setData(data);

    return serie;
}

core::IVisualizer::SerieBase* ExampleIntVisualizerStatistics::createSerie(const core::IVisualizer::SerieBase* serie)
{
    throw std::runtime_error("Not implemented");
    return nullptr;
}

void ExampleIntVisualizerStatistics::removeSerie(core::IVisualizer::SerieBase* serie)
{
    StatsSerie * s = dynamic_cast<StatsSerie*>(serie);
    widget->layout()->removeWidget(s->widget);
    s->widget = nullptr;
}

void ExampleIntVisualizerStatistics::reset()
{

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

ExampleIntVisualizerStatistics::StatsSerie::StatsSerie(QLineEdit * widget) : widget(widget)
{

}

void ExampleIntVisualizerStatistics::StatsSerie::setName(const std::string & name)
{
    this->name = name;
}

void ExampleIntVisualizerStatistics::StatsSerie::setData(const core::ObjectWrapperConstPtr & data)
{
    this->data = data;
    ExampleIntStatisticsConstPtr stats = data->get();

    QString text("Empty serie");

    if(stats != nullptr){
        text.clear();
        
        text += "Mean = " + QString::number(stats->getMean()) + " Median = " + QString::number(stats->getMedian()) + " Max = " + QString::number(stats->getMax()) + " Min = " + QString::number(stats->getMin());
    }

    widget->setText(text);
}

const std::string & ExampleIntVisualizerStatistics::StatsSerie::getName() const
{
    return name;
}

const core::ObjectWrapperConstPtr & ExampleIntVisualizerStatistics::StatsSerie::getData() const
{
    return data;
}