/********************************************************************
    created:  2013/05/15
    created:  15:5:2013   23:07
    filename: CombinationGenerator.h
    author:   Mateusz Janiak
    
    purpose:  Klasa generujaca kombinacje
*********************************************************************/
#ifndef HEADER_GUARD___COMBINATIONGENERATOR_H__
#define HEADER_GUARD___COMBINATIONGENERATOR_H__

#include <vector>
#include <boost/array.hpp>

namespace GeneralAlgorithms {

class CombinationGenerator
{

public:

	typedef std::vector<unsigned int> Combination;

private:
	const unsigned int r;
	const unsigned int n;
	const unsigned int nMinusR;
	const unsigned int rMinusOne;
	const unsigned int total;

	Combination a;
	unsigned int numLeft;

public:
	//------------
	// Constructor
	//------------
	CombinationGenerator(unsigned int r, unsigned int n);
	//------
	// Reset
	//------
	void reset ();
	//------------------------------------------------
	// Return number of combinations not yet generated
	//------------------------------------------------
	const unsigned int getNumLeft () const;
	//-----------------------------
	// Are there more combinations?
	//-----------------------------
	const bool hasMore () const;

	//--------------------------------------------------------
	// Generate next combination (algorithm from Rosen p. 286)
	//--------------------------------------------------------
	const Combination & getNext ();

	static const unsigned int getFactorial(unsigned int n);
};

template <unsigned int N>
struct TFactorial 
{
	enum { value = N * TFactorial<N - 1>::value };
};

template <>
struct TFactorial<0> 
{
	enum { value = 1 };
};

template <unsigned int R, unsigned int N>
class TCombinationGenerator
{
	static_assert((R < N), "Bledna konfiguracja kombinacji");
	static_assert((N > 1), "Bledna wielkosc zbioru wejsciowego");

public:

	typedef boost::array<unsigned int, R> Combination;

	const enum { nMinusR = N-R };
	const enum { rMinusOne = R-1 };
	const enum { total = TFactorial<N>::value / (TFactorial<R>::value * TFactorial<nMinusR>::value) };

private:
	Combination a;
	unsigned int numLeft;

public:
	//------------
	// Constructor
	//------------
	TCombinationGenerator() {
		reset ();
	}
	//------
	// Reset
	//------
	void reset () {
		for (int i = 0; i < R; i++) {
			a[i] = i;
		}
		numLeft = total;
	}
	//------------------------------------------------
	// Return number of combinations not yet generated
	//------------------------------------------------
	const unsigned int getNumLeft() const {
		return numLeft;
	}
	//-----------------------------
	// Are there more combinations?
	//-----------------------------
	const bool hasMore () const {
		return numLeft != 0;
	}

	//--------------------------------------------------------
	// Generate next combination (algorithm from Rosen p. 286)
	//--------------------------------------------------------
	const Combination & getNext () {
		if (numLeft == total) {
			--numLeft;
			return a;
		}
		unsigned int i = rMinusOne;
		while (a[i] == nMinusR + i) {
			i--;
		}
		a[i] = a[i] + 1;
		for (int j = i + 1; j < R; j++) {
			a[j] = a[i] + j - i;
		}
		--numLeft;
		return a;
	}
};

}

#endif	//	HEADER_GUARD___COMBINATIONGENERATOR_H__
