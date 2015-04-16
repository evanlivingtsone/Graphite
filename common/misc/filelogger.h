#pragma once

#include <fstream>

class FileLogger {

public:
    enum e_logType { LOG_ERROR, LOG_WARNING, LOG_INFO };
    explicit FileLogger (const char *engine_version, const char *fname = "fl_log.txt");
    ~FileLogger ();

    friend FileLogger &operator << (FileLogger &logger, const e_logType l_type);
    friend FileLogger &operator << (FileLogger &logger, const char *text);
//            FileLogger (const FileLogger &) = delete;
//            FileLogger &operator= (const FileLogger &) = delete;

private:

    std::ofstream           myFile;
    unsigned int            numWarnings;
    unsigned int            numErrors;

}; 
