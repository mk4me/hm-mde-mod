/********************************************************************
    created:  2011/06/16
    created:  16:6:2011   9:51
    filename: Plugin.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___PLUGIN_H__
#define HEADER_GUARD___PLUGIN_H__

#include <core/Plugin.h>
#include <exampleWorkflowPlugin/include/exampleIntStatistics.h>

typedef std::vector<int> Ints;
typedef core::shared_ptr<Ints> IntsPtr;
typedef core::shared_ptr<const Ints> IntsConstPtr;

CORE_DEFINE_WRAPPER(Ints, utils::PtrPolicyBoost, utils::ClonePolicyCopyConstructor);
CORE_DEFINE_WRAPPER(ExampleIntStatistics, utils::PtrPolicyBoost, utils::ClonePolicyCopyConstructor);

#endif  //  HEADER_GUARD___PLUGIN_H__
