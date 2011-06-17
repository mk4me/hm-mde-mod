#include "PCH.h"
#include "exampleIntWidgetProcessorFilterConfiguration.h"

ExampleWidgetProcessorFilterConfiguration::ExampleWidgetProcessorFilterConfiguration(ExampleIntProccesorFilter * processor)
    :processor(processor)
{
    UTILS_ASSERT((processor != nullptr), "Blêdny processor");

    setupUi(this);

    connect(minFilter, SIGNAL(pressed()), this, SLOT(radioChecked()));
    connect(equalFilter, SIGNAL(pressed()), this, SLOT(radioChecked()));
    connect(differentFilter, SIGNAL(pressed()), this, SLOT(radioChecked()));
    connect(maxFilter, SIGNAL(pressed()), this, SLOT(radioChecked()));
    connect(inRangeFilter, SIGNAL(pressed()), this, SLOT(radioChecked()));
    connect(outRangeFilter, SIGNAL(pressed()), this, SLOT(radioChecked()));
    connect(saveFilterButton, SIGNAL(pressed()), this, SLOT(saveFilter()));
}

void ExampleWidgetProcessorFilterConfiguration::radioChecked()
{
    QRadioButton * radio = qobject_cast<QRadioButton *>(sender());

    if(radio == nullptr){
        return;
    }

    if(radio == minFilter || radio == equalFilter || radio == differentFilter || radio == maxFilter){

        singleVal->setEnabled(true);
        minVal->setEnabled(false);
        maxVal->setEnabled(false);

    }else if(radio == inRangeFilter || radio == outRangeFilter){

        singleVal->setEnabled(false);
        minVal->setEnabled(true);
        maxVal->setEnabled(true);

    }
}

void ExampleWidgetProcessorFilterConfiguration::saveFilter()
{

    if(minFilter->isChecked() == true){
        
        processor->setFilter(boost::bind(&ExampleWidgetProcessorFilterConfiguration::lessThan, _1, singleVal->value()));
        
    }else if(equalFilter->isChecked() == true){
        
        processor->setFilter(boost::bind(&ExampleWidgetProcessorFilterConfiguration::equalTo, _1, singleVal->value()));
        
    }else if(differentFilter->isChecked() == true){
        
        processor->setFilter(boost::bind(&ExampleWidgetProcessorFilterConfiguration::differentThan, _1, singleVal->value()));
        
    }else if(maxFilter->isChecked() == true){

        processor->setFilter(boost::bind(&ExampleWidgetProcessorFilterConfiguration::moreThan, _1, singleVal->value()));

    }else if(inRangeFilter->isChecked() == true){
        
        processor->setFilter(boost::bind(&ExampleWidgetProcessorFilterConfiguration::inRange, _1, minVal->value(), maxVal->value()));
        
    }else if(outRangeFilter->isChecked() == true){

        processor->setFilter(boost::bind(&ExampleWidgetProcessorFilterConfiguration::outRange, _1, minVal->value(), maxVal->value()));
    }
}

bool ExampleWidgetProcessorFilterConfiguration::lessThan(int a, int val)
{
    return (a <= val);
}

bool ExampleWidgetProcessorFilterConfiguration::moreThan(int a, int val)
{
    return (a >= val);
}

bool ExampleWidgetProcessorFilterConfiguration::equalTo(int a, int val)
{
    return (a == val);
}

bool ExampleWidgetProcessorFilterConfiguration::differentThan(int a, int val)
{
    return !equalTo(a, val);
}

bool ExampleWidgetProcessorFilterConfiguration::inRange(int a, int min, int max)
{
    return (a >= min && a <= max);
}

bool ExampleWidgetProcessorFilterConfiguration::outRange(int a, int min, int max)
{
    return (a <= min || a >= max);
}