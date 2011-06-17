/********************************************************************
    created:  2011/06/16
    created:  16:6:2011   9:23
    filename: exampleIntProcessorFilter.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___EXAMPLEINTPROCESSORFILTER_H__
#define HEADER_GUARD___EXAMPLEINTPROCESSORFILTER_H__


#include <core/IDataProcessor.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <exampleWorkflowPlugin/include/exampleIntStatistics.h>

class ExampleIntProccesorFilter : public core::IDataProcessor
{
    UNIQUE_ID("{A4B67836-E8C8-4E07-A8D9-60E3089D2510}", "ExampleIntProccesorFilter");

public:
    typedef boost::function<bool (int)> DataFilter;

public:

    ExampleIntProccesorFilter(const DataFilter & comparator = boost::bind(&ExampleIntProccesorFilter::nonZeroFilter, _1));

    ExampleIntProccesorFilter(const ExampleIntProccesorFilter & processorFilter);

    virtual const std::string & getName() const;

    virtual ExampleIntProccesorFilter* createClone() const;

    virtual void process(core::IObjectSource* input, core::IObjectOutput* output);

    virtual void getInputInfo(std::vector<InputInfo>& info);

    virtual void getOutputInfo( std::vector<OutputInfo> & output );

    virtual QWidget* getConfigurationWidget();

    virtual void reset();

    void setFilter(const DataFilter & coparator);
    const DataFilter & getFilter() const;

    void setReferenceValue(int referenceValue);
    int getReferenceValue() const;

    static bool nonZeroFilter(int val);

private:

    DataFilter dataFilter;
};

#endif  //  HEADER_GUARD___EXAMPLEINTPROCESSORFILTER_H__