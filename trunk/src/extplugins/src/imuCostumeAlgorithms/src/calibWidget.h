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
    CalibWidget(bool *bindPoseFlag, bool *bowPoseFlag);

private slots:
	void bindClickedSig();
//    void radioChecked();
//    void saveFilter();

//signals:
//	void angleChanged(float angle);

private:
	bool *_bindPoseFlag;
	bool *_bowPoseFlag;
};

#endif  //  HEADER_GUARD___CALIBWIDGET_H__