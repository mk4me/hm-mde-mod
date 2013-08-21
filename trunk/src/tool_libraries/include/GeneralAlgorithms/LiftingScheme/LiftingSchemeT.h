/********************************************************************
    created:  2013/06/14
    created:  14:6:2013   10:41
    filename: LiftingSchemeT.h
    author:   Mateusz Janiak
    
    purpose:  Wzorzec klasy realizuj¹cej schemat liftingu
*********************************************************************/
#ifndef HEADER_GUARD_ALGO__LIFTINGSCHEMET_H__
#define HEADER_GUARD_ALGO__LIFTINGSCHEMET_H__

/** \file

<b>Copyright and Use</b>

You may use this source code without limitation and without
fee as long as you include:

<blockquote>
This software was written and is copyrighted by Ian Kaplan, Bear
Products International, www.bearcave.com, 2002.
</blockquote>

This software is provided "as is", without any warranty or
claim as to its usefulness.  Anyone who uses this source code
uses it at their own risk.  Nor is any support provided by
Ian Kaplan and Bear Products International.

Please send any bug fixes or suggested source changes to:
<pre>
iank@bearcave.com
</pre>

@author Ian Kaplan

*/ 

#include <boost/assert.hpp>

/**
This is the base class for simple Lifting Scheme wavelets using
split, predict, update or update, predict, merge steps.


Simple lifting scheme wavelets consist of three steps,
a split/merge step, predict step and an update step:

<ul>
<li>
<p>
The split step divides the elements in an array so that 
the even elements are in the first half and the odd
elements are in the second half.
</p>
</li>
<li>
<p>
The merge step is the Inverse of the split step.  It takes
two regions of an array, an odd region and an even region
and merges them into a new region where an even element 
alternates with an odd element.
</p>
</li>
<li>
<p>
The predict step calculates the difference
between an odd element and its predicted value based
on the even elements.  The difference between the
predicted value and the actual value replaces the
odd element.
</p>
</li>
<li>
<p>
The predict step operates on the odd elements.  The update
step operates on the even element, replacing them with a
difference between the predict value and the actual odd element.
The update step replaces each even element with an average.
The result of the update step becomes the input to the 
next recursive step in the wavelet calculation.
</p>
</li>

</ul>

The split and merge methods are shared by all Lifting Scheme wavelet
algorithms.  This base class provides the transform and Inverse
transform methods (forwardTrans and inverseTrans).  The predict
and update methods are abstract and are defined for a particular
Lifting Scheme wavelet sub-class.

This is a template version of the lifting scheme base class.  The
template must be instantiated with an array or an object that acts
like an array.  Objects that act like arrays define the left hand
side and right hand side index operators: [].  To allow wavelet
transforms based on this base class to be used with the wavelet
packet transform, this class makes public both the Forward
and Inverse transforms (forwardTrans and inverseTrans) and
the Forward and Inverse transform steps (forwardStep and
inverseStep).  These "step" functions are used to calculate
the wavelet packet transform.

<b>Instantiating the Template</b>

The Liftbase template takes two type arguments:

<ol>
<li>
The type of the array or '[]' operator indexable object.
</li>
<li>
The type of the data element.
</li>
</ol>

The simplest example is a wavelet class derived from an instance of
the Liftbase tempate which takes a double array and has a double
element type.  This declaration is shown below:

<pre>
class Haar : public Liftbase<double *, double>
</pre>

An object type can be used for the first template argument,
as long as the object supports the '[]' operator, which returns
an element whose type is defined by the second argument.  In the
example below, the packcontainer '[]' operator returns a 
double.

<pre>
class Poly : public Liftbase<packcontainer, double>
</pre>

<b>References:</b>

<ul>
<li>
<a href="http://www.bearcave.com/misl/misl_tech/wavelets/packet/index.html">
<i>The Wavelet Packet Transform</i></a> by Ian Kaplan, www.bearcave.com.
</li>
<li>
<a 
href="http://www.bearcave.com/misl/misl_tech/wavelets/lifting/index.html">
<i>The Wavelet Lifting Scheme</i></a> by Ian Kaplan, www.bearcave.com.
This is the parent web page for this Java source code.
</li>
<li>
<i>Ripples in Mathematics: the Discrete Wavelet Transform</i> 
by Arne Jense and Anders la Cour-Harbo, Springer, 2001
</li>
<li>
<i>Building Your Own Wavelets at Home</i> in <a
href="http://www.multires.caltech.edu/teaching/courses/waveletcourse/">
Wavelets in Computer Graphics</a>
</li>
</ul>

\author Ian Kaplan

*/

#include <vector>
#include <algorithm>
#include <GeneralAlgorithms/LiftingScheme/IndexHelperPolicy.h>

namespace LiftingScheme
{

class LiftingSchemeUtils
{
public:
	//! Wyliczenie opisuj¹ce kierunek transformaty liftingu
	enum TransDirection { 
		/** "enumeration" for Forward wavelet transform */
		Forward = 1,
		/** "enumeration" for Inverse wavelet transform */
		Inverse = 2 
	};

public:

	/**
	Split the <i>vec</i> into even and odd elements,
	where the even elements are in the first half
	of the vector and the odd elements are in the
	second half.
	*/

	//! \param vec Kontener z danymi o dostêpie indexowym
	//! \param N rozmiar od zera do jakiego splitujemy kontener
	template<class Data>
	static void split( Data& vec, const typename Data::size_type N ) {

		typedef typename Data::size_type size_type;

		size_type start = 1;
		size_type end = N - 1;

		while (start < end) {
			for (uint i = start; i < end; i += 2) {
				std::swap(vec[i], vec[i+1]);
			}
			start++;
			end--;
		}
	}

	/**
	Merge the odd elements from the second half of the N element
	region in the array with the even elements in the first
	half of the N element region.  The result will be the
	combination of the odd and even elements in a region
	of length N.

	*/

	//! \param vec Kontener z danymi o dostêpie indexowym
	//! \param N rozmiar od zera do jakiego mergujemy kontener
	template<class Data>
	static void merge( Data& vec, const typename Data::size_type N )
	{
		typedef typename Data::size_type size_type;

		size_type half = N >> 1;
		size_type start = half-1;
		size_type end = half;

		while (start > 0) {
			for (size_type i = start; i < end; i += 2) {
				std::swap(vec[i], vec[i+1]);
			}
			start--;
			end++;
		}
	}

	//! \param value Wartos ktora testujemy
	//! \return Czy wartoœæ jest potenga dwojki
	static const bool isPowerOfTwo(const IndexResolver::lint value) {
		return (value > 0) && !(value & (value-1));
	}

	static const double log2(const IndexResolver::lint value) {
		static const double log2value(std::log(2.0f));

		return std::log((double)value) / log2value;
	}

	//! \param value Wartoœæ testowana
	//! \return Najwieksza potega dwojki nie wieksza niz testowana wartosc
	static IndexResolver::ulint floorPowerOfTwo(const IndexResolver::lint value) {
		
		if (value <= 2)
			return value;
		auto x = (value >> 1) + 1;
		x |= x >> 1;
		x |= x >> 2;
		x |= x >> 4;
		x |= x >> 8;
		x |= x >> 16;

		return x+1;
	}

};

template<class T>
class LiftingSchemeT {

public:
	typedef std::vector<T> Data;
    typedef typename Data::size_type size_type;

protected:

	/** 
	Predict step, to be defined by the subclass

	@param vec input array
	@param N size of region to act on (from 0..N-1)
	@param direction Forward or Inverse transform

	*/
	virtual void predict( Data& vec, const size_type N,
		const LiftingSchemeUtils::TransDirection direction ) = 0;

	/**
	Reverse predict step.  

	The predict step applied the high pass filter to the data
	set and places the result in the upper half of the array.
	The reverse predict step applies the high pass filter and
	places the result in the lower half of the array.

	This reverse predict step is only used by wavelet packet
	frequency analysis algorithms.  The default version
	of this algorihtm does nothing.
	*/
	virtual void predictRev( Data& vec, const size_type N,
		const LiftingSchemeUtils::TransDirection direction ) {};


	/** 
	Update step, to be defined by the subclass 

	@param vec input array
	@param N size of region to act on (from 0..N-1)
	@param direction Forward or Inverse transform

	*/
	virtual void update( Data& vec, const size_type N,
		const LiftingSchemeUtils::TransDirection direction ) = 0;


	/**
	Reverse update step
	*/
	virtual void updateRev( Data& vec, const size_type N,
		const LiftingSchemeUtils::TransDirection direction ) {}

public:

	/**
	One step in the Forward wavelet transform
	*/
	virtual void forwardStep( Data& vec, const size_type N )
	{
		BOOST_ASSERT((LiftingSchemeUtils::isPowerOfTwo(N) && N <= vec.size()));
		LiftingSchemeUtils::split( vec, N );
		predict( vec, N, LiftingSchemeUtils::Forward );
		update( vec, N, LiftingSchemeUtils::Forward );
	} // forwardStep

	/**
	Reverse Forward transform step.  The result of the high
	pass filter is stored in the lower half of the array
	and the result of the low pass filter is stored in the
	upper half.

	This function should be defined by any subclass that
	is used for wavelet frequency analysis.
	*/
	virtual void forwardStepRev( Data& vec, const size_type N )
	{
		//BOOST_ASSERT((isPowerOfTwo(N) && N <= vec.size()));
		BOOST_ASSERT((false));
	}

	/**
	Simple wavelet Lifting Scheme Forward transform

	forwardTrans is passed an indexable object.  The object must
	contain a power of two number of data elements.  Lifting Scheme
	wavelet transforms are calculated in-place and the result is
	returned in the argument array.

	The result of forwardTrans is a set of wavelet coefficients
	ordered by increasing frequency and an approximate average
	of the input data set in vec[0].  The coefficient bands
	follow this element in powers of two (e.g., 1, 2, 4, 8...).

	*/
	virtual void forwardTrans( Data& vec, const size_type N )
	{
		BOOST_ASSERT((LiftingSchemeUtils::isPowerOfTwo(N) && N <= vec.size()));
		for (size_type n = N; n > 1; n >>= 1) {
			forwardStep( vec, n );
		}
	} // forwardTrans

	/**
	One Inverse wavelet transform step
	*/
	virtual void inverseStep( Data& vec, const size_type N )
	{
		BOOST_ASSERT((LiftingSchemeUtils::isPowerOfTwo(N) && N <= vec.size()));
		update( vec, N, LiftingSchemeUtils::Inverse );
		predict( vec, N, LiftingSchemeUtils::Inverse );
		LiftingSchemeUtils::merge( vec, N );
	}

	/** 
	Reverse Inverse transform step.  Calculate the Inverse transform
	from a high pass filter result stored in the lower half of the
	array and a low pass filter result stored in the upper half.

	This function should be defined by any subclass that
	is used for wavelet frequency analysis.
	*/
	virtual void inverseStepRev( Data& vec, const size_type N )
	{
		//BOOST_ASSERT((isPowerOfTwo(N) && N <= vec.size()));
		BOOST_ASSERT((false));
	}


	/**
	Default two step Lifting Scheme Inverse wavelet transform

	inverseTrans is passed the result of an ordered wavelet 
	transform, consisting of an average and a set of wavelet
	coefficients.  The Inverse transform is calculated
	in-place and the result is returned in the argument array.

	*/
	virtual void inverseTrans( Data& vec, const size_type N )
	{
		for (size_type n = 2; n <= N; n <<= 1) {
			inverseStep( vec, n );
		}
	} // inverseTrans


}; // Liftbase

template <class T, template<typename> class Interpolator, class IR = PeriodicIndexResolver>
class InterpolatorLiftingSchemeT : public LiftingSchemeT<T> {

protected:
	virtual void predict( Data& vec, const size_type N, const LiftingSchemeUtils::TransDirection direction ){
		Interpolator<LiftingSchemeIndexHelper<IR>>::interpolate(vec, N, direction);
	}

	virtual void update( Data& vec, const size_type N, const LiftingSchemeUtils::TransDirection direction ){
		Interpolator<LiftingSchemeIndexHelper<IR>>::update(vec, N, direction);
	}

};

}

#endif	//	HEADER_GUARD_ALGO__LIFTINGSCHEMET_H__
