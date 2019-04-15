//======================================================================//
//  This software is free: you can redistribute it and/or modify        //
//  it under the terms of the GNU General Public License Version 3,     //
//  as published by the Free Software Foundation.                       //
//  This software is distributed in the hope that it will be useful,    //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of      //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE..  See the      //
//  GNU General Public License for more details.                        //
//  You should have received a copy of the GNU General Public License   //
//  Version 3 in the file COPYING that came with this distribution.     //
//  If not, see <http://www.gnu.org/licenses/>                          //
//======================================================================//
//                                                                      //
//      Copyright (c) 2019 SinfonIA Pepper RoboCup Team                 //
//      Sinfonia - Colombia                                             //
//      https://sinfoniateam.github.io/sinfonia/index.html              //
//                                                                      //
//======================================================================//




#ifndef FILESYSTEM_HELPERS_HPP
#define FILESYSTEM_HELPERS_HPP

#include <qi/session.hpp>

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string/replace.hpp>

#ifdef CATKIN_BUILD
#include <ros/package.h>
#endif

namespace Sinfonia
{
    namespace Helpers
    {
	namespace FileSystem
	{

	   static const long folderMaximumSize = 2000000000;

	    inline void getFoldersize(std::string rootFolder, long& fileSize)
	    {
		boost::algorithm::replace_all(rootFolder, "\\\\", "\\");
		boost::filesystem::path folderPath(rootFolder);
		if (boost::filesystem::exists(folderPath))
		{
		    boost::filesystem::directory_iterator end_itr;

		    for (boost::filesystem::directory_iterator dirIte(rootFolder); dirIte != end_itr; ++dirIte )
		    {
			boost::filesystem::path filePath(dirIte->path());
			try
			{
			    if (!boost::filesystem::is_directory(dirIte->status()) )
			    {
				fileSize = fileSize + boost::filesystem::file_size(filePath);
				
			    }
			    else
			    {
				getFoldersize(filePath.string(), fileSize);
			    }
			}
			catch(std::exception& e)
			{
			    std::cout << e.what() << std::endl;
			}
		    }
		}
	    }

	    inline void getFiles(const boost::filesystem::path& root, const std::string& extension, std::vector<boost::filesystem::path>& ret)
	    {
		if(!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root)) 
		{
		    return;
		}

		boost::filesystem::recursive_directory_iterator it(root);
		boost::filesystem::recursive_directory_iterator endit;

		while(it != endit)
		{
		    if(boost::filesystem::is_regular_file(*it) && it->path().extension() == extension)
		    {
			ret.push_back(it->path().filename());
		    }
		    ++it;
		}
	    }

	    inline void getFilesSize(const boost::filesystem::path& root, long& fileSize)
	    {
		std::vector<boost::filesystem::path> filesPath;
		getFiles(root, ".bag", filesPath);
		for (std::vector<boost::filesystem::path>::const_iterator it=filesPath.begin(); it!=filesPath.end(); it++)
		{
		    try
		    {
			fileSize = fileSize + boost::filesystem::file_size(*it);
		    }
		    catch(std::exception& e)
		    {
			std::cout << e.what() << std::endl;
		    }
		}
	    }

	    static const std::string bootConfigFileName = "boot_config.json";
	    
	    inline std::string& getBootConfigFile()
	    {
		#ifdef CATKIN_BUILD
		    static std::string path = ros::package::getPath("naoqi_driver") + "/share/" + bootConfigFileName;
		    std::cout << "found a catkin prefix " << path << std::endl;
		    return path;
		#else
		    static std::string path = qi::path::findData( "/", bootConfigFileName );
		    std::cout << "found a qibuild path " << path << std::endl;
		    return path;
		#endif
	    }
	    
	    
	    inline std::string& getURDF( std::string fileName )
	    {
		#ifdef CATKIN_BUILD
		    static std::string path = ros::package::getPath("naoqi_driver")+"/share/urdf/"+fileName;
		    std::cout << "found a catkin URDF " << path << std::endl;
		return path;
		#else
		    static std::string path = qi::path::findData( "/urdf/", fileName );
		    std::cout << "found a qibuild URDF " << path << std::endl;
		    return path;
		#endif
	    }
	    
	}
    } 
} 
#endif
