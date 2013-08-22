/********************************************************************
    created:  2013/08/21
    created:  21:8:2013   14:00
    filename: EulerNoiseAdderProcessor.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD_MOTION_ANALYSIS__EULERNOISEADDERPROCESSOR_H__
#define HEADER_GUARD_MOTION_ANALYSIS__EULERNOISEADDERPROCESSOR_H__

#include "Types.h"

#include <QtCore/QObject>

#include <plugins/newVdf/INodeConfiguration.h>
#include <dflib/Node.h>
#include <dflib/IDFNode.h>

class EulerNoiseAdderProcessor : public QObject, public df::ProcessingNode,
	public df::IDFProcessor, public vdf::INodeConfiguration
{

	Q_OBJECT

public:

	EulerNoiseAdderProcessor();
	~EulerNoiseAdderProcessor();

	virtual void process();
	virtual void reset();

	virtual QWidget* getConfigurationWidget();
	virtual void refreshConfiguration();

private slots:

	void valueChanged(double val);

private:
	//! Szum
	double noise;
	VectorOutputPin* outPinA;
	VectorInputPin* inPinA;
	QWidget * widget;
};

#endif	//	HEADER_GUARD_MOTION_ANALYSIS__EULERNOISEADDERPROCESSOR_H__
