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
	JointStateRecorder::JointStateRecorder( const std::string& topic, float bufferFrequency )
	{
	    _topic =  topic;
	    _bufferDuration = Helpers::Recorder::bufferDefaultDuration;
	    _isInitialized = false;
	    _isSubscribed = false;
	    _bufferFrequency = bufferFrequency;
	    _counter = 1;
	}

	void JointStateRecorder::write( const sensor_msgs::JointState& jointStateMessage, const std::vector<geometry_msgs::TransformStamped>& TfTtransforms )
	{
	    if (!jointStateMessage.header.stamp.isZero())
	    {
		_globalRecorder->write(_topic, jointStateMessage, jointStateMessage.header.stamp);
	    }
	    else 
	    {
		_globalRecorder->write(_topic, jointStateMessage);
	    }
	    _globalRecorder->write("/tf", TfTtransforms);
	}

	void JointStateRecorder::writeDump(const ros::Time& time)
	{
	    boost::mutex::scoped_lock lock_write_buffer(_mutex);
	    boost::circular_buffer< std::vector<geometry_msgs::TransformStamped> >::iterator iTf;
	    for (iTf = _bufferTF.begin(); iTf != _bufferTF.end(); iTf++)
	    {
		_globalRecorder->write("/tf", *iTf);
	    }
	    for (boost::circular_buffer<sensor_msgs::JointState>::iterator itJs = _bufferJoinState.begin();
		itJs != _bufferJoinState.end(); itJs++)
	    {
		if (!itJs->header.stamp.isZero())
		{
		    _globalRecorder->write(_topic, *itJs, itJs->header.stamp);
		}
		else
		{
		    _globalRecorder->write(_topic, *itJs);
		}
	    }
	}

	void JointStateRecorder::reset(boost::shared_ptr<GlobalRecorder> globalRecorder, float converterFrequency)
	{
	    _globalRecorder = globalRecorder;
	    _converterFrequency = converterFrequency;
	    if (_bufferFrequency != 0)
	    {
		_maxCounter = static_cast<int>(_converterFrequency/_bufferFrequency);
		_bufferSize = static_cast<size_t>(_bufferDuration*(_converterFrequency/_maxCounter));
	    }
	    else
	    {
		_maxCounter = 1;
		_bufferSize = static_cast<size_t>(_bufferDuration*_converterFrequency);
	    }
	    _bufferJoinState.resize(_bufferSize);
	    _bufferTF.resize(_bufferSize);
	    _isInitialized = true;
	}

	void JointStateRecorder::bufferize( const sensor_msgs::JointState& jointStateMessage, const std::vector<geometry_msgs::TransformStamped>& tfTransforms )
	{
	    boost::mutex::scoped_lock lock_bufferize( _mutex );
	    if (_counter < _maxCounter)
	    {
		_counter++;
	    }
	    else
	    {
		_counter = 1;
		_bufferJoinState.push_back(jointStateMessage);
		_bufferTF.push_back(tfTransforms);
	    }
	}

	void JointStateRecorder::setBufferDuration(float duration)
	{
	    boost::mutex::scoped_lock lock_bufferize(_mutex);
	    _bufferSize = static_cast<size_t>(duration*(_converterFrequency/_maxCounter));
	    _bufferDuration = duration;
	    _bufferJoinState.set_capacity(_bufferSize);
	    _bufferTF.set_capacity(_bufferSize);
	}

    }
}
