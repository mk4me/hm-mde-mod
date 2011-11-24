/********************************************************************
	created:	2011/05/25
	created:	25:5:2011   10:14
	filename: 	C3DParser.h
	author:		Wojciech Knieæ
	
	purpose:	
*********************************************************************/

#ifndef HEADER_GUARD_C3DMARKERS__C3DPARSER_H__
#define HEADER_GUARD_C3DMARKERS__C3DPARSER_H__

#include <core/SmartPtr.h>
#include <core/Filesystem.h>
#include <core/IParser.h>
#include <core/IDataManager.h>
#include <c3dlib/C3DParser.h>
#include <utils/DataChannel.h>


//! parser wczytuje plik c3d i zwraca obiekt typu AllMarkersCollection (zawiera wszystkie markery zawarte w pliku)
class C3DMarkersParser : public core::IParser, utils::GeneralDataChannelTimeAccessor<osg::Vec3f, float>
{
    UNIQUE_ID("{B6E91A06-A22B-4FB1-839D-7764645BAE27}", "C3DMarkersParser");
private:
	core::ObjectWrapperPtr allmarkerCollection;
	core::shared_ptr<c3dlib::C3DParser> parserPtr;
    	
public:
    C3DMarkersParser();
    virtual ~C3DMarkersParser();

public:
    virtual void parseFile(const core::Filesystem::Path& path);
    virtual core::IParser* create();
    virtual void getSupportedExtensions(core::IParser::Extensions & extensions) const;    
    virtual void getObjects(core::Objects& objects);
    void saveFile(const core::Filesystem::Path& path);
};


#endif