/********************************************************************
    created:  2011/06/28
    created:  28:6:2011   8:17
    filename: exampleScalarConfigurationWidget.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___EXAMPLESCALARCONFIGURATIONWIDGET_H__
#define HEADER_GUARD___EXAMPLESCALARCONFIGURATIONWIDGET_H__

#include "ui_exampleScalarConfigurationWidget.h"
#include <QtGui/QWidget>

class ExampleScalarChannelProcessor;

class ExampleScalarConfigurationWidget : public QWidget, public Ui::exampleScalarConfiguration
{
    Q_OBJECT

public:
    ExampleScalarConfigurationWidget(ExampleScalarChannelProcessor * processor, QWidget * parent = nullptr);
    ~ExampleScalarConfigurationWidget();

protected slots:
    void scaleChanged(double scale);

private:
    ExampleScalarChannelProcessor * processor;
};


#endif HEADER_GUARD___EXAMPLESCALARCONFIGURATIONWIDGET_H__
