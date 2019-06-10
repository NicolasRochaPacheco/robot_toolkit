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



#ifndef SONAR_PUBLISHER_HPP
#define SONAR_PUBLISHER_HPP

#include <string>

#include <sensor_msgs/Range.h>
#include <ros/ros.h>

namespace Sinfonia
{
    namespace Publisher
    {

	class SonarPublisher
	{

	    public:
		SonarPublisher(std::vector<std::string> topicNames);
		virtual ~SonarPublisher() {}
		std::string getTopicName();


		bool isInitialized() const;

		bool isSubscribed() const;

		void publish( const std::vector<sensor_msgs::Range>& message );

		void reset( ros::NodeHandle& nodeHandle );
		
		void shutdown();

	    protected:
		bool _isInitialized;
		std::vector<ros::Publisher> _publishers;
		std::vector<std::string> _topicNames;
	};
    } 
} 

#endif
