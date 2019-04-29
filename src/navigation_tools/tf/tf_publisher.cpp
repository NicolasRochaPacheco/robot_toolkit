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


#include "tf_publisher.hpp"

namespace Sinfonia
{
    namespace Publisher
    {

	TfPublisher::TfPublisher()
	{
	    _isInitialized = false;
	    _topic = "/tf";
	}
	
	std::string TfPublisher::topic()
	{
	    return _topic;
	}
	
	bool TfPublisher::isInitialized()
	{
	    return _isInitialized;
	}

	void TfPublisher::publish(const std::vector< geometry_msgs::TransformStamped >& TfTransforms)
	{
	    _TFBroadcasterPtr->sendTransform(TfTransforms);
	}

	void TfPublisher::reset( ros::NodeHandle& nodeHandle )
	{
	    _TFBroadcasterPtr = boost::make_shared<tf2_ros::TransformBroadcaster>();
	    _isInitialized = true;
	}

	bool TfPublisher::isSubscribed() const
	{
	    return true;
	}

    }
} 