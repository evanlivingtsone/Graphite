
#include <fstream>
#include "filelogger.h"

FileLogger::FileLogger (const char *engine_version, const char *fname) : numWarnings (0U), numErrors (0U)
{

	myFile.open (fname);

	// Write the first lines
	if (myFile.is_open()) {
	myFile << "My Game Engine, version " << engine_version << std::endl;
	myFile << "Log file created" << std::endl << std::endl;
	} // if

}


FileLogger::~FileLogger () 
{

	if (myFile.is_open()) {
	    myFile << std::endl << std::endl;

	    // Report number of errors and warnings
	    myFile << numWarnings << " warnings" << std::endl;
	    myFile << numErrors << " errors" << std::endl;

	    myFile.close();
	} // if

}


// Overload << operator using log type
FileLogger& operator<<(FileLogger &logger, const FileLogger::e_logType l_type) {

	switch (l_type) {
	    case FileLogger::LOG_ERROR:
		logger.myFile << "[ERROR]: ";
		++logger.numErrors;
		break;

	    case FileLogger::LOG_WARNING:
		logger.myFile << "[WARNING]: ";
		++logger.numWarnings;
		break;

	    default:
		logger.myFile << "[INFO]: ";
		break;
	} // sw


	return logger; 
}


// Overload << operator using C style strings
// No need for std::string objects here
FileLogger& operator<<(FileLogger &logger, const char *text) {

	logger.myFile << text << std::endl;
	return logger;

}





