/********************************************************************
	created:	2013/03/21
	created:	21:3:2013   13:31
	filename: 	exampleIntParser.h
	author:		Wojciech Kniec
	
	purpose:	
*********************************************************************/

#ifndef HEADER_GUARD_EXAMPLE__EXAMPLEINTPARSER_H__
#define HEADER_GUARD_EXAMPLE__EXAMPLEINTPARSER_H__


#include <corelib/Filesystem.h>
#include <corelib/IParser.h>

class ExampleIntParser : public plugin::ISourceParser
{
    UNIQUE_ID("{A4B67836-E8C8-4E77-A8D9-60E3089D2510}");
	CLASS_DESCRIPTION("Example Int Parser", "Example Int Parser")

public:
	ExampleIntParser();

	virtual void acceptedExpressions( Expressions & expressions ) const;
	virtual plugin::IParser* create() const;
	virtual void parse( const std::string & source );

	virtual void getObject(core::Variant& object, const core::VariantsVector::size_type idx) const;

	virtual void reset();

private:
    utils::ObjectWrapperPtr adapter;
};


#endif