#include "PCH.h"
//#include "examplePins.h"
//
//
//void ExampleIntOutputPin::reset()
//{
//	val = IntsPtr();
//}
//
//void ExampleIntOutputPin::value( IntsPtr val )
//{
//	this->val = val;
//}
//
//const IntsConstPtr ExampleIntOutputPin::value() const
//{
//	return val;
//}
//
//ExampleIntOutputPin::ExampleIntOutputPin( df::ISourceNode * node ) : df::OutputPin(node)
//{
//
//}
//
//const IntsConstPtr ExampleIntInputPin::value() const
//{
//	return val;
//}
//
//void ExampleIntInputPin::reset()
//{
//	val = IntsPtr();
//}
//
//void ExampleIntInputPin::copyData( const df::IDFOutput * pin )
//{
//	IntsConstPtr ints = dynamic_cast<const ExampleIntOutputPin*>(pin)->value();
//	val = IntsPtr(new Ints(*ints));
//}
//
//
//ExampleIntInputPin::ExampleIntInputPin( df::ISinkNode * node ) : df::InputPin(node)
//{
//
//}
//
//const bool ExampleIntInputPin::pinCompatible( const df::IOutputPin * pin ) const
//{
//    return dynamic_cast<const ExampleIntOutputPin*>(pin);
//}
//
//void ExampleStatsOutputPin::reset()
//{
//	val = ExampleIntStatisticsPtr();
//}
//
//void ExampleStatsOutputPin::value( ExampleIntStatisticsPtr val )
//{
//	this->val = val;
//}
//
//const ExampleIntStatisticsConstPtr ExampleStatsOutputPin::value() const
//{
//	return val;
//}
//
//ExampleStatsOutputPin::ExampleStatsOutputPin( df::ISourceNode * node ) : df::OutputPin(node)
//{
//
//}
