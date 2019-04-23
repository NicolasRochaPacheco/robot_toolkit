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


#include "navigation_tools.hpp"

namespace Sinfonia
{
    namespace Publisher
    {

	NavigationToolsPublisher::NavigationToolsPublisher()
	{
	    isInitialized_ = false;
	}

	void NavigationToolsPublisher::publishTF(const std::vector<geometry_msgs::TransformStamped>& TfTransforms )
	{
	    TFBroadcasterPtr_->sendTransform(TfTransforms);
	}


	void NavigationToolsPublisher::reset( ros::NodeHandle& nodeHandle )
	{

	    TFBroadcasterPtr_ = boost::make_shared<tf2_ros::TransformBroadcaster>();

	    isInitialized_ = true;
	}

	bool NavigationToolsPublisher::isSubscribed() const
	{
	    return true;
	}

    }
} 
