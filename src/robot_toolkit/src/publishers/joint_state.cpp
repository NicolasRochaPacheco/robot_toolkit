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


#include "joint_state.hpp"

namespace Sinfonia
{
    namespace Publisher
    {

	JointStatePublisher::JointStatePublisher( const std::string& topic ):
	topic_( topic ),
	isInitialized_( false )
	{}

	void JointStatePublisher::publish( const sensor_msgs::JointState& jointStatesMessage,
					const std::vector<geometry_msgs::TransformStamped>& TfTransforms )
	{
	    pubJointStates_.publish(jointStatesMessage);
	    TFBroadcasterPtr_->sendTransform(TfTransforms);
	}


	void JointStatePublisher::reset( ros::NodeHandle& nodeHandle )
	{
	    pubJointStates_ = nodeHandle.advertise<sensor_msgs::JointState>( topic_, 10 );

	    TFBroadcasterPtr_ = boost::make_shared<tf2_ros::TransformBroadcaster>();

	    isInitialized_ = true;
	}

	bool JointStatePublisher::isSubscribed() const
	{
	    return true;
	}

    }
} 
