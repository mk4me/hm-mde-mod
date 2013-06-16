#include <IMU/Parsers/FileCSVParser.h>
#include <sstream>

using namespace IMU;

FileCSVParser::FileCSVParser(const std::string & path, const bool cache, const char delimiter, const char escape, const char quota)
	: lastResult_(READ_OK)
{
	boost::shared_ptr<std::ifstream> fileStream(new std::ifstream(path)); 

	if(fileStream->is_open() == true){		

		if(cache == true){		

			fileStream->seekg(0,std::ios::end);
			std::streampos length = fileStream->tellg();			
			if((std::streamsize)length > 0){
				fileStream->seekg(0,std::ios::beg);
				// Get a vector that size and
				std::vector<char> buf(length);
				// Fill the buffer with the size
				fileStream->read(&buf[0],length);

				stream_.reset(new std::stringstream(std::string(buf.begin(), buf.end())));
			}else{
				lastResult_ = ICSVParser::READ_FINISHED;
			}

			fileStream->close();
		}else{
			stream_ = fileStream;
		}
		
		if(stream_ != nullptr){
			streamParser_.reset(new StreamCSVParser(stream_.get(), delimiter, escape, quota));		
		}
	}else{
		lastResult_ = ICSVParser::READ_IO_ERROR;
	}
}

FileCSVParser::~FileCSVParser()
{
	if(stream_ != nullptr){
		auto fStream = boost::dynamic_pointer_cast<std::ifstream>(stream_);
		if(fStream != nullptr && fStream->is_open() == true){
			fStream->close();
		}
	}
}

const FileCSVParser::ReadResultType FileCSVParser::readNext(LineData & lineData)
{
	if(lastResult_ == ICSVParser::READ_OK){

		lastResult_ = streamParser_->readNext(lineData);

	}

	return lastResult_;
}

const FileCSVParser::ReadResultType FileCSVParser::skipNext(const LineData::size_type n)
{
	if(lastResult_ == ICSVParser::READ_OK){

		lastResult_ = streamParser_->skipNext(n);

	}

	return lastResult_;
}