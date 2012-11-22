/********************************************************************
    created:  2011/06/16
    created:  16:6:2011   13:28
    filename: exampleIntWidgetProcessorFilterConfiguration.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___EXAMPLEINTWIDGETPROCESSORFILTERCONFIGURATION_H__
#define HEADER_GUARD___EXAMPLEINTWIDGETPROCESSORFILTERCONFIGURATION_H__

#include "ui_exampleIntWidgetProcessorFilterConfiguration.h"
#include "exampleIntProcessorFilter.h"

class ExampleWidgetProcessorFilterConfiguration : public QWidget, public Ui::FilterConfiguration
{
    Q_OBJECT;

public:
    ExampleWidgetProcessorFilterConfiguration(ExampleIntProccesorFilter * processor);

private slots:
    void radioChecked();
    void saveFilter();

private:
    static bool lessThan(int a, int val);
    static bool moreThan(int a, int val);
    static bool equalTo(int a, int val);
    static bool differentThan(int a, int val);

    static bool inRange(int a, int min, int max);
    static bool outRange(int a, int min, int max);


private:
    ExampleIntProccesorFilter * processor;
};

#endif  //  HEADER_GUARD___EXAMPLEINTWIDGETPROCESSORFILTERCONFIGURATION_H__