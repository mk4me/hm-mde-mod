/********************************************************************
    created:  2013/06/14
    created:  14:6:2013   10:46
    filename: BorderedLiftingSchemeT.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD_ALGO__BORDEREDLIFTINGSCHEMET_H__
#define HEADER_GUARD_ALGO__BORDEREDLIFTINGSCHEMET_H__

#include <boost/assert.hpp>

namespace LiftingScheme
{

//! Klasa bazowa, definiuje podstawowe typy
class IndexResolver
{
public:
	typedef unsigned long int ulint;
	typedef long int lint;
};

//! Klasa pomocnicza przy generowaniu indeks�w pr�bek
//! Zak�ada okresowo�� sygna�u - dla pr�bki mniejszej od 0 generuje pr�bk� od ko�ca przedzia�u maj�c na uwadze d�ugo�� przedzia�u (modulo)
//! dla pr�bki wi�kszej od size - 1 generuje pr�bk� od pocz�tku przedzia�u maj�c na uwadze d�ugo�� przedzia�u (modulo)
class PeriodicIndexResolver : public IndexResolver
{
public:
    inline static const ulint indexUnderflow(const lint idx, const ulint max,
		const ulint min)
    {
        BOOST_ASSERT((max - min) > 0);
        BOOST_ASSERT(idx < min);
		const ulint diff = max - min;
		return (diff == 1) ? min : max - ((min - idx) % diff);
    }

    inline static const ulint indexOverflow(const lint idx, const ulint max,
		const ulint min)
    {
        BOOST_ASSERT((max - min) > 0);
        BOOST_ASSERT(idx >= max);
        return min + (idx - max) % (max - min);
    }
};

//! Klasa pomocnicza przy generowaniu indeks�w pr�bek
//! Dla indeks�w mniejszych od 0 zwraca 0 a dla wi�kszych lub r�wnych size zwraca size - 1
class BorderIndexResolver : public IndexResolver
{
public:
    inline static const ulint indexUnderflow(const lint idx, const ulint max,
		const ulint min)
    {
		BOOST_ASSERT((max - min) > 0);
        BOOST_ASSERT(idx < min);
        return min;
    }

    inline static const ulint indexOverflow(const lint idx, const ulint max,
		const ulint min)
    {
		BOOST_ASSERT((max - min) > 0);
        BOOST_ASSERT(idx >= max);
        return max - 1;
    }
};

template<class IR = LiftingScheme::PeriodicIndexResolver>
class LiftingSchemeIndexHelper
{
private:
	typedef IR IndexResolverType;

public:

	inline static IndexResolver::ulint index(const IndexResolver::lint idx,
		const IndexResolver::ulint max, const IndexResolver::ulint min = 0)
	{
		IndexResolver::ulint ret = idx;
		if(idx < min){
			ret = IndexResolverType::indexUnderflow(idx, max, min);
		}else if(idx >= max){
			ret = IndexResolverType::indexOverflow(idx, max, min);
		}

		BOOST_ASSERT((ret >= min && ret < max), "B�edny indeks");

		return ret;
	}
};

}

#endif	//	HEADER_GUARD_ALGO__BORDEREDLIFTINGSCHEMET_H__
