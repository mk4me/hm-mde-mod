/********************************************************************
    created:  2011/06/16
    created:  16:6:2011   9:23
    filename: exampleIntStatistics.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___EXAMPLEINTSTATISTICS_H__
#define HEADER_GUARD___EXAMPLEINTSTATISTICS_H__


#include <utils/Debug.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/kurtosis.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/type_traits.hpp>
#include <utils/SmartPtr.h>

namespace acc = boost::accumulators;

template<class T>
class ExampleStatistics
{
    UTILS_STATIC_ASSERT((boost::is_arithmetic<T>::value), "Template class must be arithmetic type! For custom types propose custom statistic class");

public:
    ExampleStatistics()
    {
    }

    ExampleStatistics(const ExampleStatistics & statistics) : accumulator(statistics.accumulator)
    {
    }

    void addSample(T sample)
    {
        accumulator(sample);
    }

    T getMean() const
    {
        return acc::mean(accumulator);
    }

    T getSecondMoment() const
    {
        return acc::moment<2>(accumulator);
    }

    T getKurtosis() const
    {
        return acc::kurtosis(accumulator);
    }

    T getMax() const
    {
        return (acc::max)(accumulator);
    }

    T getMin() const
    {
        return (acc::min)(accumulator);
    }

    T getMedian() const
    {
        return acc::median(accumulator);
    }

    T getSkewness() const
    {
        return acc::skewness(accumulator);
    }

private:
    acc::accumulator_set<T, acc::stats<acc::tag::mean, acc::tag::moment<2>,
        acc::tag::kurtosis, acc::tag::max, acc::tag::min, acc::tag::median, acc::tag::skewness> > accumulator;
};

typedef ExampleStatistics<int> ExampleIntStatistics;

typedef utils::shared_ptr<ExampleIntStatistics> ExampleIntStatisticsPtr;
typedef utils::shared_ptr<const ExampleIntStatistics> ExampleIntStatisticsConstPtr;

#endif  //  HEADER_GUARD___EXAMPLEINTSTATISTICS_H__
