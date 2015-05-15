/********************************************************************
    created:  04.05.2015
    filename: calibWidget.h
    author:   Kamil Lebek
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___CALIBWIDGET_H__
#define HEADER_GUARD___CALIBWIDGET_H__

#include "ui_calib.h"

class CalibWidget : public QWidget, public Ui_calibForm
{
    Q_OBJECT;

public:
	typedef enum { CS_START, CS_BINDPOSE, CS_BOWPOSE } ECalibStage;
	//! Simple constructor
	CalibWidget(ECalibStage& _calibStage);

private slots:
	void bindClickedSig();
	void bowClickedSig();

//signals:
//	void angleChanged(float angle);

private:
	ECalibStage& _calibStage;

};

#endif  //  HEADER_GUARD___CALIBWIDGET_H__