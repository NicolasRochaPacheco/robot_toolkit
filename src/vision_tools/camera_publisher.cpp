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

#include "robot_toolkit/vision_tools/camera_publisher.hpp"

namespace Sinfonia
{
    namespace Publisher
    {
	
	CameraPublisher::CameraPublisher(std::string topic)
	{
	    _topic = topic;
	}
	
	std::string CameraPublisher::topic()
	{
	    return _topic;
	}
	
	bool CameraPublisher::isInitialized()
	{
	    return _isInitialized;
	}

	void CameraPublisher::publish(const sensor_msgs::ImagePtr& img, const sensor_msgs::CameraInfo& cameraInfo)
	{
	    _publisher.publish(*img, cameraInfo);
	}
	
	void CameraPublisher::reset(ros::NodeHandle& nodeHandle)
	{
	    image_transport::ImageTransport it( nodeHandle );
	    _publisher = it.advertiseCamera( _topic, 1 );

	    if ( _cameraSource != Helpers::VisionHelpers::kDepthCamera)
	    {
		std::string nodeName = ros::this_node::getName();
		XmlRpc::XmlRpcValue args, result, payload;
		args[0] = nodeName;
		args[1] = nodeName;
		ros::master::execute("lookupNode", args, result, payload, false);
		args[2] = result[2];
		
		std::vector<std::string> topicList;
		topicList.push_back(std::string("/") + nodeName + "/" + _topic + std::string("/compressedDepth"));
		topicList.push_back(std::string("/") + nodeName + "/" + _topic + std::string("/compressedDepth/parameter_updates"));
		topicList.push_back(std::string("/") + nodeName + "/" + _topic + std::string("/compressedDepth/parameter_descriptions"));

		for(std::vector<std::string>::const_iterator topic = topicList.begin(); topic != topicList.end(); ++topic)
		{
		    args[1] = *topic;
		    ros::master::execute("unregisterPublisher", args, result, payload, false);
		}
	    }

	    _isInitialized = true;

	}

	bool CameraPublisher::isSubscribed() const
	{
	    if (!_isInitialized) 
		return false;
	    return _publisher.getNumSubscribers() > 0;
	}
	
	void CameraPublisher::shutdown()
	{
	    _publisher.shutdown();
	}
	
	void CameraPublisher::setCameraSource(int cameraSource)
	{
	    _cameraSource = cameraSource;
	}

    }
}