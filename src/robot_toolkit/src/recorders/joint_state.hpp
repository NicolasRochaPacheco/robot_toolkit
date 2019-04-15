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

#ifndef JOINT_STATE_RECORDER_HPP
#define JOINT_STATE_RECORDER_HPP

/*
* BOOST includes
*/
#include <boost/circular_buffer.hpp>

/*
* LOCAL includes
*/
#include "robot_toolkit/recorder/global_recorder.hpp"
#include "../helpers/recorder_helpers.hpp"

/*
* ROS includes
*/
#include <sensor_msgs/JointState.h>

namespace Sinfonia 
{
    namespace Recorder
    {
	class JointStateRecorder
	{
	    public:
		JointStateRecorder( const std::string& topic, float bufferFrequency = 0 );

		void write( const sensor_msgs::JointState& jointStateMessage,
			    const std::vector<geometry_msgs::TransformStamped>& TfTtransforms );

		void reset( boost::shared_ptr<Sinfonia::Recorder::GlobalRecorder> globalRecorder, float converterFrequency );

		void bufferize( const sensor_msgs::JointState& jointStateMessage,
				const std::vector<geometry_msgs::TransformStamped>& TfTransforms );

		void writeDump(const ros::Time& time);

		void setBufferDuration(float duration);

		inline std::string topic() const
		{
		    return topic_;
		}

		inline bool isInitialized() const
		{
		    return isInitialized_;
		}

		inline void subscribe( bool state)
		{
		    isSubscribed_ = state;
		}

		inline bool isSubscribed() const
		{
		    return isSubscribed_;
		}

	    protected:
		std::string topic_;

		boost::circular_buffer<sensor_msgs::JointState> bufferJoinState_;
		boost::circular_buffer< std::vector<geometry_msgs::TransformStamped> > bufferTF_;
		size_t bufferSize_;
		float bufferDuration_;

		boost::mutex mutex_;

		bool isInitialized_;
		bool isSubscribed_;

		boost::shared_ptr<Sinfonia::Recorder::GlobalRecorder> globalRecorder_;

		float bufferFrequency_;
		float converterFrequency_;
		int counter_;
		int maxCounter_;

	};

    }
}

#endif
