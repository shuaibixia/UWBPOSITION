
#include "csv.h"

void    InitDFile()
{
	std::string       fileNM = { "./uwbsim.csv" }; 
	std::stringstream traceLog;

	logging::add_file_log
    (
        keywords::file_name     = fileNM.c_str(),
        keywords::rotation_size = 10 * 1024 * 1024,
        keywords::auto_flush    = true 
    );

	traceLog << "id,tagId,baseId,dist,";
	BOOST_LOG_TRIVIAL (trace) << (traceLog.str ().c_str());
}