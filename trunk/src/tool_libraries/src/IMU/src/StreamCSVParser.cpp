#include <IMU/Parsers/StreamCSVParser.h>
#include <boost/tokenizer.hpp>

using namespace IMU;

StreamCSVParser::StreamCSVParser(std::istream * stream, const char delimiter, const char escape, const char quota)
	: stream_(stream), lastResult_(READ_OK), delimiter_(delimiter), escape_(escape), quota_(quota)
{
	if(stream_ == nullptr){
		throw std::invalid_argument("Null stream pointer for parsing");
	}

	updateStreamStatus();
}

StreamCSVParser::~StreamCSVParser()
{

}

const StreamCSVParser::ReadResultType StreamCSVParser::readNext(LineData & lineData)
{
	ReadResultType ret = lastResult_;

	if(ret == ICSVParser::READ_OK){

		std::string line;
		std::getline(*stream_, line);

		updateStreamStatus();

		if(lastResult_ == ICSVParser::READ_OK){

			using namespace boost;
			typedef tokenizer<escaped_list_separator<char>> TokenizerType;
			TokenizerType tk(line, escaped_list_separator<char>(escape_, delimiter_, quota_));						
			auto endIT = tk.end();

			for(auto it = tk.begin(); it != endIT; ++it){
				lineData.push_back(*it);
			}

		}else{
			ret = lastResult_;
		}
	}

	return ret;
}

const StreamCSVParser::ReadResultType StreamCSVParser::skipNext(const LineData::size_type n)
{
	for(LineData::size_type i = 0; i < n && lastResult_ == ICSVParser::READ_OK; ++i) {
		std::string line;
		std::getline(*stream_, line);
		updateStreamStatus();
	}

	return lastResult_;
}

void StreamCSVParser::updateStreamStatus()
{
	if(stream_->eof() == true){
		lastResult_ = ICSVParser::READ_FINISHED;
	}else if(stream_->fail() == true || stream_->bad() == true){
		lastResult_ = ICSVParser::READ_IO_ERROR;
	}
}