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


#ifndef CAMERA_PUBLISHER_HPP
#define CAMERA_PUBLISHER_HPP

#include <image_transport/image_transport.h>
#include "robot_toolkit/helpers/vision_helpers.hpp"

namespace Sinfonia
{
    namespace Publisher
    {

	class CameraPublisher
	{

	    public:
		CameraPublisher(std::string topicName);
		std::string getTopicName();

		bool isInitialized();
		
		virtual void publish(const sensor_msgs::ImagePtr img, const sensor_msgs::CameraInfoPtr cameraInfo);
		virtual void reset( ros::NodeHandle& nodeHandle );
		virtual bool isSubscribed() const;
		virtual void shutdown();
		
		void setCameraSource(int cameraSource);

	    private:
		
		int _cameraSource;
		
		image_transport::CameraPublisher _publisher;

		std::string _topicName;

		bool _isInitialized;

	};

    }
}

#endif