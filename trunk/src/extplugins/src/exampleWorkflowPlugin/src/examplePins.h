/********************************************************************
	created:	2013/03/25
	created:	25:3:2013   11:18
	filename: 	examplePins.h
	author:		Wojciech Kniec
	
	purpose:	
*********************************************************************/

#ifndef HEADER_GUARD_EXAMPLE__EXAMPLEPINS_H__
#define HEADER_GUARD_EXAMPLE__EXAMPLEPINS_H__

#include <plugins/dfElements/DFPins.h>
#include "Plugin.h"

typedef UniversalOutputPinT<ExampleIntStatistics> StatsOutputPin;
typedef UniversalInputPinT<Ints> ExampleIntInputPin;
typedef UniversalOutputPinT<Ints> ExampleIntOutputPin;

//#include <dflib/Pin.h>
//#include <dflib/IDFPin.h>
//#include <dflib/IDFNode.h>
//#include <dflib/Node.h>
//#include <plugins/newVdf/IDataFlowProvider.h>
//#include "Plugin.h"
//
//class ExampleIntOutputPin : public df::OutputPin, public df::IDFOutput, public vdf::IMDEOutputPin
//{
//    MDE_OUTPUT_PIN(Ints);
//public:
//	ExampleIntOutputPin(df::ISourceNode * node);
//
//public:
//	const IntsConstPtr value() const;
//	void value(IntsPtr val);
//	virtual void reset();
//
//private:
//	IntsPtr val;
//};
//
//
//class ExampleIntInputPin : public df::InputPin, public df::IDFInput
//{
//public:
//	ExampleIntInputPin(df::ISinkNode * node);
//	
//public:
//	virtual void copyData(const df::IDFOutput * pin);
//
//    virtual void reset();
//	const IntsConstPtr value() const;
//
//    virtual const bool pinCompatible( const df::IOutputPin * pin ) const;
//
//private:
//	IntsPtr val;
//};
//
//class ExampleStatsOutputPin : public df::OutputPin, public df::IDFOutput, public vdf::IMDEOutputPin
//{
//    MDE_OUTPUT_PIN(ExampleIntStatistics)
//public:
//	ExampleStatsOutputPin(df::ISourceNode * node);
//
//public:
//	const ExampleIntStatisticsConstPtr value() const;
//	void value(ExampleIntStatisticsPtr val);
//	virtual void reset();
//
//private:
//	ExampleIntStatisticsPtr val;
//};

#endif
