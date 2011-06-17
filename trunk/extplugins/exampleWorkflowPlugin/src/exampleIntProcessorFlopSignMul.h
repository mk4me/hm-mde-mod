/********************************************************************
    created:  2011/06/16
    created:  16:6:2011   9:34
    filename: exampleIntProcessorFlopSignMul.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___EXAMPLEINTPROCESSORFLOPSIGNMUL_H__
#define HEADER_GUARD___EXAMPLEINTPROCESSORFLOPSIGNMUL_H__


#include <exampleWorkflowPlugin/include/exampleIntStatistics.h>
#include <core/IDataProcessor.h>


class ExampleIntProccesorFlopSignMul : public core::IDataProcessor
{
    UNIQUE_ID("{A4B67836-E8C8-4E07-A8D9-60E3089D2599}", "ExampleIntProccesorFlopSignMul");

public:

    virtual const std::string & getName() const;

    virtual ExampleIntProccesorFlopSignMul* createClone() const;

    virtual void process(core::IObjectSource* input, core::IObjectOutput* output);

    virtual void getInputInfo(std::vector<InputInfo>& info);

    virtual void getOutputInfo( std::vector<OutputInfo> & output );

    virtual QWidget* getConfigurationWidget();

    virtual void reset();

};

#endif  //  HEADER_GUARD___EXAMPLEINTPROCESSORFLOPSIGNMUL_H__