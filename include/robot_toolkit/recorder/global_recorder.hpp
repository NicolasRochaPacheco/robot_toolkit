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

#ifndef GLOBALRECORDER_HPP
#define GLOBALRECORDER_HPP

#include "robot_toolkit/tools.hpp"

#include <string>

#include <boost/thread/mutex.hpp>


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/TransformStamped.h>

namespace Sinfonia
{
    namespace Recorder
    {
	class GlobalRecorder
	{

	    public:

		GlobalRecorder(const std::string& prefixTopic);
		void startRecord(const std::string& prefixBag = "");
		std::string stopRecord(const std::string& robotIp = "<ROBOT_IP>");

		template <class T>
		void write(const std::string& topic, const T& message, const ros::Time& time = ros::Time::now() )
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
		    ros::Time timeMessage = time;
		    boost::mutex::scoped_lock writeLock( _processMutex );
		    if (_isStarted)
		    {
			_bag.write(rosTopic, timeMessage, message);
		    }
		}

		void write(const std::string& topic, const std::vector<geometry_msgs::TransformStamped>& messsagetf);
		bool isStarted();

	    private:
		std::string _prefixTopic;
		boost::mutex _processMutex;
		rosbag::Bag _bag;
		std::string _nameBag;
		bool _isStarted;

		std::vector<Topics> _topics;

	};
    }
} 

#endif
