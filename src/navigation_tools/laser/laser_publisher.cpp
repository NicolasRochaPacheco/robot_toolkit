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

#include "laser_publisher.hpp"

namespace Sinfonia
{
    namespace Publisher
    {
	
	LaserPublisher::LaserPublisher()
	{
	    _isInitialized = false;
	     _topic = "/laser";
	}

	bool LaserPublisher::isInitialized() const
	{
	    return _isInitialized;
	}
	
	std::string LaserPublisher::topic()
	{
	    return _topic;
	}

	bool LaserPublisher::isSubscribed() const
	{
	    if (!_isInitialized) 
		return false;
	    return _publisher.getNumSubscribers() > 0;
	}

	void LaserPublisher::publish(sensor_msgs::LaserScan& message)
	{
	    _publisher.publish( message );
	}
	
	void LaserPublisher::reset(ros::NodeHandle& nodeHandle)
	{
	    _publisher = nodeHandle.advertise<sensor_msgs::LaserScan>( _topic, 10 );
	    _isInitialized = true;
	}
	
	void LaserPublisher::shutdown()
	{
	    _publisher.shutdown();
	}

    }
}
