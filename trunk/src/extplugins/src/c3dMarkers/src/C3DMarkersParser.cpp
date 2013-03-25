#include "C3DMarkersPCH.h"
#include "C3DMarkersParser.h"
#include <string>
#include <vector>
#include <c3dlib/c3dparser.h>
#include <c3dMarkers/C3DMarkersChannels.h>

C3DMarkersParser::C3DMarkersParser() : 
	allmarkerCollection(core::ObjectWrapper::create<AllMarkersCollection>())
{

}

C3DMarkersParser::~C3DMarkersParser()
{

}

void C3DMarkersParser::acceptedExpressions( Expressions & expressions ) const
{
	plugin::IParser::ExpressionDescription expDesc;
	expDesc.description = "C3D format";
	expDesc.types.insert(typeid(AllMarkersCollection));
	expressions[".*\\.c3d$"] = expDesc;
}

plugin::IParser* C3DMarkersParser::create() const
{
	return new C3DMarkersParser();
}

void C3DMarkersParser::getObjects( core::Objects& objects )
{
	objects.insert(allmarkerCollection);
}

void C3DMarkersParser::parse( const std::string & source )
{
	core::shared_ptr<c3dlib::C3DParser> parser(new c3dlib::C3DParser());
	core::Filesystem::Path path(source);
	std::vector<std::string> files;
	files.push_back(path.string());
	std::string importWarnings;
	parser->importFrom(files, importWarnings);
	AllMarkersCollectionPtr markers(new AllMarkersCollection); 
		
	int markersCount = parser->getNumPoints();
	for (int i = 0; i < markersCount; i++) {
	    MarkerChannelPtr ptr(new MarkerChannel(*parser, i)); 
	    markers->addChannel(ptr);						 
	}
	allmarkerCollection->set(markers);
}
