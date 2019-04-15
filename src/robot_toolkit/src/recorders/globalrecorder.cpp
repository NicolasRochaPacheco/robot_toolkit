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


//#include "recorder/global_recorder.hpp"
#include "robot_toolkit/recorder/global_recorder.hpp"
#include <ctime>
#include <sstream>


#include <std_msgs/Int32.h>
#include <tf2_msgs/TFMessage.h>
#include <ros/ros.h>
//#include <rosbag/bag.h>
//#include <rosbag/view.h>
#include <geometry_msgs/TransformStamped.h>


#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>


#include <qi/log.hpp>

qiLogCategory("ros.Recorder");

namespace Sinfonia
{
    namespace Recorder
    {

	GlobalRecorder::GlobalRecorder(const std::string& prefixTopic):
	_bag(),
	_processMutex(),
	_nameBag(""),
	_isStarted(false)
	{
	    if (!prefixTopic.empty())
	    {
		_prefixTopic = "/" + prefixTopic + "/";
	    }
	    else
	    {
		_prefixTopic = "/";
	    }
	}

	void GlobalRecorder::startRecord(const std::string& prefixBag)
	{
	    boost::mutex::scoped_lock startLock( _processMutex );
	    if (!_isStarted) 
	    {
		try 
		{    
		    boost::filesystem::path cur_path(boost::filesystem::current_path());
		    time_t rawTime;
		    struct tm * timeInfo;
		    char buffer[80];
		    std::time(&rawTime);
		    timeInfo = std::localtime(&rawTime);
		    std::strftime(buffer,80,"%d-%m-%Y_%I:%M:%S", timeInfo);

		    if (!prefixBag.empty()) 
		    {
			_nameBag = cur_path.string()+ "/" + prefixBag + "_" + buffer;
		    }
		    else 
		    {
			_nameBag = cur_path.string() + "/" + buffer;
		    }
		    
		    _nameBag.append(".bag");
		    _bag.open(_nameBag, rosbag::bagmode::Write);
		    _isStarted = true;
		    std::cout << YELLOW << "The bag " << BOLDCYAN << _nameBag << RESETCOLOR << YELLOW << " is opened" << RESETCOLOR << std::endl;
		    
		} 
		catch (std::exception e)
		{
		    throw std::runtime_error(e.what());
		}
	    }
	    else 
	    {
		qiLogError() << "Cannot start a record. The module is already recording.";
	    }
	}

	std::string GlobalRecorder::stopRecord(const std::string& robotIp) 
	{
	    boost::mutex::scoped_lock stopLock(_processMutex);
	    if (_isStarted) 
	    {
		//_bag.close();
		_isStarted = false;

		std::stringstream message;
		message << _nameBag;
		std::cout << YELLOW << "The bag " << BOLDCYAN << _nameBag << RESETCOLOR << YELLOW << " is closed" << RESETCOLOR << std::endl;
		char* currentPath;
		currentPath = getenv("HOME");
		std::string currentPathStr = currentPath;
		
		if (!(currentPathStr.find("nao") == std::string::npos)) 
		{
		    std::cout << BOLDRED << "To download this bag on your computer:" << RESETCOLOR << std::endl << GREEN << "\t$ scp nao@" << robotIp << ":" << _nameBag << " <LOCAL_PATH>" << RESETCOLOR << std::endl;
		}
		
		_nameBag.clear();
		return message.str();
	    }
	    else 
	    {
		qiLogError() << "Cannot stop recording while it has not been started.";
		return "Cannot stop recording while it has not been started.";
	    }
	}

	bool GlobalRecorder::isStarted() 
	{
	    return _isStarted;
	}

	void GlobalRecorder::write(const std::string& topic, const std::vector<geometry_msgs::TransformStamped>& messsagetf) 
	{
	    if (!messsagetf.empty())
	    {
		std::string rosTopic;
		if (topic[0]!='/')
		{
		    rosTopic = _prefixTopic+topic;
		}
		else
		{
		    rosTopic = topic;
		}
		tf2_msgs::TFMessage message;
		ros::Time now = ros::Time::now();
		if (!messsagetf[0].header.stamp.isZero()) 
		{
		    now = messsagetf[0].header.stamp;
		}
		for (std::vector<geometry_msgs::TransformStamped>::const_iterator it = messsagetf.begin(); it != messsagetf.end(); ++it)
		{
		    message.transforms.push_back(*it);
		}
		boost::mutex::scoped_lock writeLock( _processMutex );
		if (_isStarted) 
		{
		    _bag.write(rosTopic, now, message);
		}
	    }
	}
    } 
} 
