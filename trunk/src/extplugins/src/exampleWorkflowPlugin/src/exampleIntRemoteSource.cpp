#include "PCH.h"
#include "exampleIntRemoteSource.h"
#include "Plugin.h"

void ExampleIntRemoteSource::addVal()
{
    model->addValue(spinBox->value());
}

void ExampleIntRemoteSource::refreshConfiguration()
{

}

QWidget* ExampleIntRemoteSource::getConfigurationWidget()
{
    return widget;
}

void ExampleIntRemoteSource::produce()
{
    outPinA->setValue(ints);
    processed = true;
}

const bool ExampleIntRemoteSource::empty() const
{
    return processed;
}

void ExampleIntRemoteSource::reset()
{
    processed = false;
}

ExampleIntRemoteSource::~ExampleIntRemoteSource()
{

}

ExampleIntRemoteSource::ExampleIntRemoteSource() : 
processed(false)
{
    outPinA = new ExampleIntOutputPin(this);
    addOutputPin(outPinA);
    view = new QListView();
    widget = new QWidget();
    clearButton = new QPushButton();
    clearButton->setText("Clear");
    addValueButton = new QPushButton();
    addValueButton->setText("Add Value");


    spinBox = new QSpinBox();
    spinBox->setRange((std::numeric_limits<int>::min)(), (std::numeric_limits<int>::max)());

    QLayout* layout1 = new QHBoxLayout();
    layout1->addWidget(spinBox);
    layout1->addWidget(addValueButton);
    layout1->addWidget(clearButton);
    QWidget* cont1 = new QWidget();
    cont1->setLayout(layout1);

    QLayout* layout2 = new QVBoxLayout();
    layout2->addWidget(view);
    layout2->addWidget(cont1);
    widget->setLayout(layout2);

    ints = IntsPtr(new Ints());
    ints->push_back(20);
    ints->push_back(10);
    ints->push_back(203);
    ints->push_back(140);
    ints->push_back(9);
    ints->push_back(20);
    model = new IntsModel(ints);

    QObject::connect(clearButton, SIGNAL(clicked()), model, SLOT(clear()));
    QObject::connect(addValueButton, SIGNAL(clicked()), this, SLOT(addVal()));

    view->setModel(model);
}
