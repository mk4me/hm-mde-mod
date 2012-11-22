/********************************************************************
    created:  2011/06/16
    created:  16:6:2011   9:24
    filename: exampleIntProcessorStatistic.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___EXAMPLEINTPROCESSORSTATISTIC_H__
#define HEADER_GUARD___EXAMPLEINTPROCESSORSTATISTIC_H__


#include <exampleWorkflowPlugin/exampleIntStatistics.h>
#include <core/IDataProcessor.h>


class ExampleIntProccesorStatistic : public core::IDataProcessor
{
    UNIQUE_ID("{A4B67836-E8C8-4E07-A8D9-60E3089D2557}", "ExampleIntProccesorStatistic");

public:

    virtual const std::string & getName() const;

    virtual ExampleIntProccesorStatistic* createClone() const;

    virtual void process(core::IObjectSource* input, core::IObjectOutput* output);

    virtual void getInputInfo(std::vector<InputInfo>& info);

    virtual void getOutputInfo( std::vector<OutputInfo> & output );

    virtual QWidget* getConfigurationWidget();

    virtual void reset();

};

#endif  //  HEADER_GUARD___EXAMPLEINTPROCESSORSTATISTIC_H__