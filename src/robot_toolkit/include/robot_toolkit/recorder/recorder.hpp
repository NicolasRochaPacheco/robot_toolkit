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


#ifndef RECORDER_HPP
#define RECORDER_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/TransformStamped.h>

#include "robot_toolkit/recorder/global_recorder.hpp"
#include "robot_toolkit/tools.hpp"

namespace Sinfonia
{
    namespace Recorder
    {
	class Recorder
	{
	    public:
		
		template<typename T>
		Recorder( T recorder ):
		recorderPtr( boost::make_shared<RecorderModel<T> >(recorder) )
		{}

		bool isInitialized() const
		{
		    return recorderPtr->isInitialized();
		}


		void subscribe( bool state )
		{
		    recorderPtr->subscribe(state);
		}

		bool isSubscribed() const
		{
		    return recorderPtr->isSubscribed();
		}

		std::string topic() const
		{
		    return recorderPtr->topic();
		}

		void reset( boost::shared_ptr<Sinfonia::Recorder::GlobalRecorder> globalRecorder, float frequency)
		{
		    recorderPtr->reset( globalRecorder, frequency );
		}

		void writeDump(const ros::Time& time)
		{
		    recorderPtr->writeDump(time);
		}

		void setBufferDuration(float duration)
		{
		    recorderPtr->setBufferDuration(duration);
		}

		friend bool operator==( const Recorder& lhs, const Recorder& rhs )
		{
		    if ( lhs.topic() == rhs.topic() )
		    return true;
		    return false;
		}

	    private:

		struct RecorderConcept
		{
		    virtual ~RecorderConcept(){}
		    virtual bool isInitialized() const = 0;
		    virtual void subscribe(bool state) = 0;
		    virtual bool isSubscribed() const = 0;
		    virtual std::string topic() const = 0;
		    virtual void writeDump(const ros::Time& time) = 0;
		    virtual void setBufferDuration(float duration) = 0;
		    virtual void reset( boost::shared_ptr<Sinfonia::Recorder::GlobalRecorder> gr, float frequency ) = 0;
		};


		
		template<typename T>
		struct RecorderModel : public RecorderConcept
		{
		    RecorderModel( const T& other ):
		    recorder_( other )
		    {}

		    void reset( boost::shared_ptr<Sinfonia::Recorder::GlobalRecorder> gr, float frequency )
		    {
		    recorder_->reset( gr, frequency );
		    }

		    bool isInitialized() const
		    {
		    return recorder_->isInitialized();
		    }

		    void subscribe(bool state)
		    {
		    recorder_->subscribe( state );
		    }

		    bool isSubscribed() const
		    {
		    return recorder_->isSubscribed();
		    }

		    std::string topic() const
		    {
		    return recorder_->topic();
		    }

		    void writeDump(const ros::Time& time)
		    {
		    recorder_->writeDump(time);
		    }

		    void setBufferDuration(float duration)
		    {
		    recorder_->setBufferDuration(duration);
		    }

		    T recorder_;
		};

		boost::shared_ptr<RecorderConcept> recorderPtr;

	};
    } 
} 

#endif
