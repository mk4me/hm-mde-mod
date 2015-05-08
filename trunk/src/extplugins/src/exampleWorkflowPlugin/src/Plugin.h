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
#include <utils/PtrPolicyStd.h>
#include <utils/ClonePolicies.h>
#include <vector>

typedef std::vector<int> Ints;
typedef utils::shared_ptr<Ints> IntsPtr;
typedef utils::shared_ptr<const Ints> IntsConstPtr;

DEFINE_WRAPPER(Ints, utils::PtrPolicyStd, utils::ClonePolicyCopyConstructor);
DEFINE_WRAPPER(ExampleIntStatistics, utils::PtrPolicyStd, utils::ClonePolicyCopyConstructor);



#endif  //  HEADER_GUARD___PLUGIN_H__
