#ifndef MOTU_LOGGER
#define MOTU_LOGGER

#include <iostream>

namespace motu {
	std::ostream &writeLog();

	void setLogFile(const char *path);
}

#define LOG(args) writeLog() << args << std::endl

#endif