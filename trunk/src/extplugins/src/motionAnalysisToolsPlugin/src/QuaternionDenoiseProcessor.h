/********************************************************************
    created:  2013/08/21
    created:  21:8:2013   15:31
    filename: QuaternionDenoiseProcessor.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD_MOTION_ANALYSIS__QUATERNIONDENOISEPROCESSOR_H__
#define HEADER_GUARD_MOTION_ANALYSIS__QUATERNIONDENOISEPROCESSOR_H__

#include "Types.h"

#include <QtCore/QObject>

#include <plugins/newVdf/INodeConfiguration.h>
#include <dflib/Node.h>
#include <dflib/IDFNode.h>

class QuaternionDenoiseProcessor: public QObject, public df::ProcessingNode,
	public df::IDFProcessor, public vdf::INodeConfiguration
{

	Q_OBJECT

public:

	QuaternionDenoiseProcessor();
	~QuaternionDenoiseProcessor();

	virtual void process();
	virtual void reset();

	virtual QWidget* getConfigurationWidget();
	virtual void refreshConfiguration();

private slots:

	void valueChanged(double val);

private:
	//! Próg
	double threshold;
	JointAnglesOutputPin* outPinA;
	JointAnglesInputPin* inPinA;
	QWidget * widget;
};

#endif	//	HEADER_GUARD_MOTION_ANALYSIS__QUATERNIONDENOISEPROCESSOR_H__
