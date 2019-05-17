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




#ifndef AUDIO_EVENT_HPP
#define AUDIO_EVENT_HPP

#include <ros/ros.h>
#include <qi/session.hpp>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/enable_shared_from_this.hpp>

#include "mic_publisher.hpp"
#include "mic_converter.hpp"

namespace Sinfonia
{
    class MicEventRegister: public boost::enable_shared_from_this<MicEventRegister>
    {
	public:
	   MicEventRegister(const std::string& name, const float& frecuency, const qi::SessionPtr& session);
	   ~MicEventRegister();
	   
	   void resetPublisher(ros::NodeHandle& nodeHandle);
	   void shutdownPublisher();
	   
	   void startProcess();
	   void stopProcess();
	   void isPublishing(bool state);
	   
	   bool isStarted();
	   
	   void setDefaultParameters();
	   void setParameters(std::vector<int> parameters);
	   
	   void processRemote(int numberOfChannels, int samplesByChannel, qi::AnyValue timestamp, qi::AnyValue buffer);
	   
	private:
	    
	    void registerCallback();
	    void unregisterCallback();
	    
	    boost::shared_ptr<Converter::MicConverter> _converter;
	    boost::shared_ptr<Publisher::MicPublisher> _publisher;
	    

	    qi::SessionPtr _session;
	    qi::AnyObject _pAudio;
	    qi::AnyObject _pRobotModel;
	    qi::FutureSync<qi::AnyObject> _pAudioExtractorRequest;
	    std::vector<uint8_t> _channelMap;
	    unsigned int _serviceId;

	    boost::mutex _subscriptionMutex;
	    boost::mutex _processingMutex;

	    bool _isStarted;
	    bool _isPublishing;
	    
	    int _micSampleRate;
	    int _channels;
    };
    QI_REGISTER_OBJECT(MicEventRegister, processRemote)
}


#endif