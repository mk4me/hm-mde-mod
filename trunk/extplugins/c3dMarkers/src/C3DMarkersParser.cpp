#include "C3DMarkersPCH.h"
#include "C3DMarkersParser.h"
#include <string>
#include <vector>
#include <c3dlib/c3dparser.h>
#include <c3dMarkers/include/C3DMarkersChannels.h>

C3DMarkersParser::C3DMarkersParser()
{
	allmarkerCollection = core::ObjectWrapper::create<AllMarkersCollection>();
}

C3DMarkersParser::~C3DMarkersParser()
{
}

void C3DMarkersParser::parseFile( core::IDataManager* dataManager, const core::Filesystem::Path& path )
{
	core::shared_ptr<c3dlib::C3DParser> parser(new c3dlib::C3DParser());
    
    std::vector<const std::string> files;
    files.push_back(path.string());
	std::string importWarnings;
    parser->importFrom(files, importWarnings);
  	AllMarkersCollectionPtr markers(new AllMarkersCollection); 
	
	int markersCount = parser->getNumPoints();
	for (int i = 0; i < markersCount; i++) {
        MarkerChannelPtr ptr(new MarkerChannel(*parser, i)); 
        markers->addChannel(ptr);						 
	}
	allmarkerCollection->set(markers, path.filename().string(), path.string());
}

core::IParser* C3DMarkersParser::create()
{
    return new C3DMarkersParser();
}

std::string C3DMarkersParser::getSupportedExtensions() const
{
    return "c3d";
}

void C3DMarkersParser::getObjects( std::vector<core::ObjectWrapperPtr>& objects )
{
	objects.push_back(allmarkerCollection);
}

void C3DMarkersParser::saveFile( const core::Filesystem::Path& path )
{
	if (parserPtr) {
		parserPtr->save(path.string());
	}
}

