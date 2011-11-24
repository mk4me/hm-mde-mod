#include "PCH.h"
#include "exampleIntVisualizer.h"

const std::string& ExampleIntVisualizer::getName() const
{
    static const std::string name("ExampleIntVisualizer");
    return name;
}

core::IVisualizer* ExampleIntVisualizer::createClone() const
{
    return new ExampleIntVisualizer();
}

void ExampleIntVisualizer::getInputInfo(std::vector<core::IInputDescription::InputInfo>& info)
{
    core::IInputDescription::InputInfo input;
    input.name = "INT";
    input.modify = false;
    input.required = true;
    input.type = typeid(Ints);

    info.push_back(input);
}

void ExampleIntVisualizer::update(double deltaTime)
{

}

QWidget* ExampleIntVisualizer::createWidget(std::vector<QObject*>& actions)
{
    widget = new QWidget();
    QVBoxLayout * layout = new QVBoxLayout();
    widget->setLayout(layout);
    return widget;
}

QIcon* ExampleIntVisualizer::createIcon()
{
    //return new QIcon(getPluginResourceString("icons/ints.png"));
    return nullptr;
}

void ExampleIntVisualizer::setUp(core::IObjectSource* source)
{
    reset();
}

int ExampleIntVisualizer::getMaxDataSeries() const
{
    return -1;
}

core::IVisualizer::SerieBase* ExampleIntVisualizer::createSerie(const core::ObjectWrapperConstPtr & data, const std::string & name)
{
    QLineEdit* lineEdit = new QLineEdit();
    widget->layout()->addWidget(lineEdit);
    IntSerie * serie = new IntSerie(lineEdit);
    serie->setName(name);
    serie->setData(data);

    return serie;
}

core::IVisualizer::SerieBase* ExampleIntVisualizer::createSerie(const core::IVisualizer::SerieBase* serie)
{
    throw std::runtime_error("Not implemented");
    return nullptr;
}

void ExampleIntVisualizer::removeSerie(core::IVisualizer::SerieBase* serie)
{
    IntSerie * s = dynamic_cast<IntSerie*>(serie);
    widget->layout()->removeWidget(s->widget);
    s->widget = nullptr;
}

void ExampleIntVisualizer::reset()
{

}

ExampleIntVisualizer::IntSerie::IntSerie(QLineEdit * widget) : widget(widget)
{

}

void ExampleIntVisualizer::IntSerie::setName(const std::string & name)
{
    this->name = name;
}

void ExampleIntVisualizer::IntSerie::setData(const core::ObjectWrapperConstPtr & data)
{
    this->data = data;
    IntsConstPtr ints = data->get();

    QString text("Empty serie");

    if(ints != nullptr){
        text.clear();
        for(auto it = ints->begin(); it != ints->end(); it++){
            text += QString::number(*it) + " ";
        }

        text = text.trimmed();
    }

    widget->setText(text);

}

const std::string & ExampleIntVisualizer::IntSerie::getName() const
{
    return name;
}

const core::ObjectWrapperConstPtr & ExampleIntVisualizer::IntSerie::getData() const
{
    return data;
}