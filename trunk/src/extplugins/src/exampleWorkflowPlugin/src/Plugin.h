/********************************************************************
    created:  2011/06/16
    created:  16:6:2011   9:51
    filename: Plugin.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___PLUGIN_H__
#define HEADER_GUARD___PLUGIN_H__

#include <corelib/IPlugin.h>
#include <exampleWorkflowPlugin/exampleIntStatistics.h>
#include <plugins/newVdf/IDataFlowProvider.h>

typedef std::vector<int> Ints;
typedef core::shared_ptr<Ints> IntsPtr;
typedef core::shared_ptr<const Ints> IntsConstPtr;

DEFINE_WRAPPER(Ints, utils::PtrPolicyBoost, utils::ClonePolicyCopyConstructor);
DEFINE_WRAPPER(ExampleIntStatistics, utils::PtrPolicyBoost, utils::ClonePolicyCopyConstructor);



#endif  //  HEADER_GUARD___PLUGIN_H__
