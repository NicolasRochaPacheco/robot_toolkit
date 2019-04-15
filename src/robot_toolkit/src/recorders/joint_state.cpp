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
    namespace Recorder
    {
	JointStateRecorder::JointStateRecorder( const std::string& topic, float bufferFrequency ):
	topic_( topic ),
	bufferDuration_(Helpers::Recorder::bufferDefaultDuration),
	isInitialized_( false ),
	isSubscribed_( false ),
	bufferFrequency_(bufferFrequency),
	counter_(1)
	{
	    
	}

	void JointStateRecorder::write( const sensor_msgs::JointState& jointStateMessage, const std::vector<geometry_msgs::TransformStamped>& tf_transforms )
	{
	    if (!jointStateMessage.header.stamp.isZero())
	    {
		globalRecorder_->write(topic_, jointStateMessage, jointStateMessage.header.stamp);
	    }
	    else 
	    {
		globalRecorder_->write(topic_, jointStateMessage);
	    }
	    globalRecorder_->write("/tf", tf_transforms);
	}

	void JointStateRecorder::writeDump(const ros::Time& time)
	{
	    boost::mutex::scoped_lock lock_write_buffer(mutex_);
	    boost::circular_buffer< std::vector<geometry_msgs::TransformStamped> >::iterator iTf;
	    for (iTf = bufferTF_.begin(); iTf != bufferTF_.end(); iTf++)
	    {
		globalRecorder_->write("/tf", *iTf);
	    }
	    for (boost::circular_buffer<sensor_msgs::JointState>::iterator itJs = bufferJoinState_.begin();
		itJs != bufferJoinState_.end(); itJs++)
	    {
		if (!itJs->header.stamp.isZero())
		{
		    globalRecorder_->write(topic_, *itJs, itJs->header.stamp);
		}
		else
		{
		    globalRecorder_->write(topic_, *itJs);
		}
	    }
	}

	void JointStateRecorder::reset(boost::shared_ptr<GlobalRecorder> globalRecorder, float converterFrequency)
	{
	    globalRecorder_ = globalRecorder;
	    converterFrequency_ = converterFrequency;
	    if (bufferFrequency_ != 0)
	    {
		maxCounter_ = static_cast<int>(converterFrequency_/bufferFrequency_);
		bufferSize_ = static_cast<size_t>(bufferDuration_*(converterFrequency_/maxCounter_));
	    }
	    else
	    {
		maxCounter_ = 1;
		bufferSize_ = static_cast<size_t>(bufferDuration_*converterFrequency_);
	    }
	    bufferJoinState_.resize(bufferSize_);
	    bufferTF_.resize(bufferSize_);
	    isInitialized_ = true;
	}

	void JointStateRecorder::bufferize( const sensor_msgs::JointState& jointStateMessage, const std::vector<geometry_msgs::TransformStamped>& tfTransforms )
	{
	    boost::mutex::scoped_lock lock_bufferize( mutex_ );
	    if (counter_ < maxCounter_)
	    {
		counter_++;
	    }
	    else
	    {
		counter_ = 1;
		bufferJoinState_.push_back(jointStateMessage);
		bufferTF_.push_back(tfTransforms);
	    }
	}

	void JointStateRecorder::setBufferDuration(float duration)
	{
	    boost::mutex::scoped_lock lock_bufferize(mutex_);
	    bufferSize_ = static_cast<size_t>(duration*(converterFrequency_/maxCounter_));
	    bufferDuration_ = duration;
	    bufferJoinState_.set_capacity(bufferSize_);
	    bufferTF_.set_capacity(bufferSize_);
	}

    }
}
