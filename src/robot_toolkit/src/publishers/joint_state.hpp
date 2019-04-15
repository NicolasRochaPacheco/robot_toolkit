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


#ifndef JOINT_STATES_PUBLISHER_HPP
#define JOINT_STATES_PUBLISHER_HPP


#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>

namespace Sinfonia
{
    namespace Publisher
    {

	class JointStatePublisher
	{

	public:
	    JointStatePublisher( const std::string& topic = "/joint_states" );

	    inline std::string topic() const
	    {
		return topic_;
	    }

	    inline bool isInitialized() const
	    {
		return isInitialized_;
	    }

	    virtual void publish( const sensor_msgs::JointState& jointStatesMessage,
				    const std::vector<geometry_msgs::TransformStamped>& TfTransforms );

	    virtual void reset( ros::NodeHandle& nodeHandle );

	    virtual bool isSubscribed() const;

	private:
	    boost::shared_ptr<tf2_ros::TransformBroadcaster> TFBroadcasterPtr_;

	    ros::Publisher pubJointStates_;

	    std::string topic_;

	    bool isInitialized_;

	};

    }
}

#endif
