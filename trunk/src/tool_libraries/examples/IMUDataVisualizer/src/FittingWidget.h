/********************************************************************
    created:  2013/06/27
    created:  27:6:2013   11:57
    filename: FittingWidget.h
    author:   Mateusz Janiak
    
    purpose:  Klasa obs³uguj¹ca widget dopasowuj¹cy dane XSENS i VICON
*********************************************************************/
#ifndef HEADER_GUARD___FITTINGWIDGET_H__
#define HEADER_GUARD___FITTINGWIDGET_H__

#include <QtGui/QWidget>
#include "ui_FittingWidget.h"

class FittingWidget : public QWidget, public Ui::FittingWidget
{
	Q_OBJECT

public:
	FittingWidget(QWidget * parent = nullptr, Qt::WindowFlags f = 0);
	virtual ~FittingWidget();

};


#endif	//	HEADER_GUARD___FITTINGWIDGET_H__
