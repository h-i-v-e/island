#include "logger.h"
#include <memory>
#include <fstream>

namespace {
	struct LogHandler {
		std::unique_ptr<std::ofstream> logFile;
	} logHandler;
}

std::ostream &motu::writeLog() {
	return logHandler.logFile ? *logHandler.logFile.get() : std::cout;
}

void motu::setLogFile(const char *path) {
	logHandler.logFile = std::make_unique<std::ofstream>(path, std::ios::out | std::ios::app);
}