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


#include "mic_publisher.hpp"

namespace Sinfonia
{
    namespace Publisher
    {
	
	MicPublisher::MicPublisher()
	{
	    _isInitialized = false;
	    _topic = "/mic";
	}
	
	bool MicPublisher::isInitialized() const
	{
	    return _isInitialized;
	}
	
	std::string MicPublisher::topic()
	{
	    return _topic;
	}
	
	bool MicPublisher::isSubscribed() const
	{
	    if (!_isInitialized) 
		return false;
	    return _publisher.getNumSubscribers() > 0;
	}
	void MicPublisher::publish(naoqi_bridge_msgs::AudioBuffer& message)
	{
	    _publisher.publish( message );
	}
	void MicPublisher::reset(ros::NodeHandle& nodeHandle)
	{
	    _publisher = nodeHandle.advertise<naoqi_bridge_msgs::AudioBuffer>( _topic, 10 );
	    _isInitialized = true;
	}
	void MicPublisher::shutdown()
	{
	    _publisher.shutdown();
	}
    }
}
    