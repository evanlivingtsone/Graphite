#include "filelogger.h"

int main()
{
    // Create object
    FileLogger myLog ("1.0.4.2", "testfile.txt");

    // Writing warnings or errors to file is very easy and C++ style
    myLog << FileLogger::LOG_WARNING << "Hey! ... This is a warning message!";
    myLog << FileLogger::LOG_ERROR << "WOW! Something really wrong is happening here!";
    myLog << "This is just a simple text";

    return 0;

}
