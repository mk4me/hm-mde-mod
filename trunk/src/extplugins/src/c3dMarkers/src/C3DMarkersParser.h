/********************************************************************
	created:	2011/05/25
	created:	25:5:2011   10:14
	filename: 	C3DParser.h
	author:		Wojciech Knieæ
	
	purpose:	
*********************************************************************/

#ifndef HEADER_GUARD_C3DMARKERS__C3DPARSER_H__
#define HEADER_GUARD_C3DMARKERS__C3DPARSER_H__

#include <utils/SmartPtr.h>
#include <corelib/Filesystem.h>
#include <corelib/IParser.h>
#include <c3dlib/C3DParser.h>
#include <utils/DataChannel.h>


//! parser wczytuje plik c3d i zwraca obiekt typu AllMarkersCollection (zawiera wszystkie markery zawarte w pliku)
class C3DMarkersParser : public plugin::IParser, public plugin::ISourceParserCapabilities
{
    UNIQUE_ID("{B6E91A06-A22B-4FB1-839D-7764645BAE27}")
	CLASS_DESCRIPTION("C3DMarkersParser", "C3DMarkersParser");
private:
	utils::ObjectWrapperPtr allmarkerCollection;
	core::shared_ptr<c3dlib::C3DParser> parserPtr;
    	
public:
    C3DMarkersParser();
    virtual ~C3DMarkersParser();

public:
	virtual void acceptedExpressions( Expressions & expressions ) const;
	virtual plugin::IParser* create() const;
	virtual void getObjects( core::Objects& objects );
	virtual void parse( const std::string & source );
};


#endif