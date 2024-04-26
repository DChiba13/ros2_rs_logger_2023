#ifndef _PROJECT_RYUSEI_LIB_FILE_PROCESSOR_GET_HOME_DIR_HPP_
#define _PROJECT_RYUSEI_LIB_FILE_PROCESSOR_GET_HOME_DIR_HPP_

//-----------------------------
// include
//-----------------------------
// UNIX
#include <stdlib.h>
#include <pwd.h>
#include <unistd.h>
#include <stdio.h>

//-----------------------------
// Namespace
//-----------------------------

//-----------------------------
// Functions
//-----------------------------
namespace project_ryusei{
inline std::string get_homedir()
{
    return (std::string)"/home/" + getpwuid(geteuid())->pw_name + "/";
}
}

#endif // _PROJECT_RYUSEI_LIB_FILE_PROCESSOR_CREATE_DIRECTORY_HPP_

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------