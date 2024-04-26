#ifndef _PROJECT_RYUSEI_LIB_FILE_PROCESSOR_CREATE_DIRECTORY_HPP_
#define _PROJECT_RYUSEI_LIB_FILE_PROCESSOR_CREATE_DIRECTORY_HPP_

//-----------------------------
// include
//-----------------------------
// Boost
#include <boost/filesystem.hpp>

//-----------------------------
// Namespace
//-----------------------------
namespace fs = boost::filesystem;

//-----------------------------
// Functions
//-----------------------------
/**
 * @brief Create a Directory object
 * 
 * @param dir full path
 * @return true  
 * @return false 
 */
inline bool createDirectory(const std::string &dir)
{
    boost::system::error_code error;
    bool result = fs::create_directories(fs::path(dir), error);
    return (result && !error);
}

#endif // _PROJECT_RYUSEI_LIB_FILE_PROCESSOR_CREATE_DIRECTORY_HPP_

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------