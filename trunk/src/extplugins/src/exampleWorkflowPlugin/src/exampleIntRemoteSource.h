/********************************************************************
	created:	2013/04/02
	created:	2:4:2013   12:26
	filename: 	exampleIntRemoteSource.h
	author:		Wojciech Kniec
	
	purpose:	
*********************************************************************/

#ifndef HEADER_GUARD_EXAMPLE__EXAMPLEINTREMOTESOURCE_H__
#define HEADER_GUARD_EXAMPLE__EXAMPLEINTREMOTESOURCE_H__


#include <corelib/PluginCommon.h>
#include <dflib/Pin.h>
#include <dflib/IDFPin.h>
#include <dflib/IDFNode.h>
#include <dflib/Node.h>
#include <boost/function.hpp>
#include <corelib/ILog.h>
#include <plugins/newVdf/INodeConfiguration.h>
#include <QtWidgets/QLabel>
#include <QtCore/QAbstractListModel>
#include <QtWidgets/QListView>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <limits>
#include "IntsModel.h"
#include "examplePins.h"


class ExampleIntRemoteSource: public QObject, public df::SourceNode, public df::IDFSource, public vdf::INodeConfiguration
{
    Q_OBJECT;
public:
    ExampleIntRemoteSource();
    virtual ~ExampleIntRemoteSource();

    virtual void reset();

    virtual const bool empty() const;

    virtual void produce();

    virtual QWidget* getConfigurationWidget();
    virtual void refreshConfiguration(); 

private slots:
    void addVal();

private:
    ExampleIntOutputPin * outPinA;
    QListView* view;
    QPushButton* clearButton;
    QPushButton* addValueButton;
    QSpinBox* spinBox;
    QWidget* widget;
    IntsModel* model;
    IntsPtr ints;
    bool processed;
};
typedef utils::shared_ptr<ExampleIntRemoteSource> exampleIntRemoteSourcePtr;
typedef utils::shared_ptr<const ExampleIntRemoteSource> exampleIntRemoteSourceConstPtr;


#endif
