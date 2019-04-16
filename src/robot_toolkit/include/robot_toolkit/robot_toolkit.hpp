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

#ifndef ROBOT_TOOLKIT_HPP
#define ROBOT_TOOLKIT_HPP

#include <qi/session.hpp>
#include <boost/thread/mutex.hpp>

#include "robot_toolkit/ros_environment.hpp"
#include "robot_toolkit/converter/converter.hpp"
#include "robot_toolkit/publisher/publisher.hpp"
#include "robot_toolkit/recorder/recorder.hpp"

#include <tf2_ros/buffer.h>

namespace tf2_ros
{
    class Buffer;
}


namespace Sinfonia
{
    class RobotToolkit
    {
	public:
	    RobotToolkit(qi::SessionPtr session, const std::string& prefix);
	    ~RobotToolkit();
	    
	    std::string _whoWillWin();
	    void init();
	    void stopService();
	    void setMasterURINet(const std::string& uri, const std::string& networkInterface);
	    void startPublishing();

	    
	private:
	    
	    struct ScheduledConverter 
	    {
		ScheduledConverter(const ros::Time& schedule, size_t conv_index):
		schedule_(schedule), conv_index_(conv_index)
		{
		}

		bool operator < (const ScheduledConverter& sp_in) const 
		{
		    return schedule_ > sp_in.schedule_;
		}
		
		ros::Time schedule_;
		
		size_t conv_index_;
	    };
	    
	    boost::thread mainThread;
	    bool _publishEnabled;
	    bool _recordEnabled;
	    bool _logEnabled;
	    //std::map< std::string, event::Event > eventMap;
	    bool isRosLoopEnabled;
	    boost::scoped_ptr<ros::NodeHandle> nodeHandlerPtr;
	    boost::mutex mutexConvertersQueue;
	    boost::mutex mutexRecorders;
	    qi::SessionPtr sessionPtr;
	    boost::shared_ptr<tf2_ros::Buffer> tf2Buffer;
	    std::vector< Converter::Converter > _converters;
	    std::priority_queue<ScheduledConverter> _convertersQueue;
	    
	    //std::map< std::string, event::Event > eventMap_;
	    
	    boost::shared_ptr<Recorder::GlobalRecorder> _recorder;
	    std::map< std::string, Recorder::Recorder > _recorderMap;
	    std::map< std::string, Publisher::Publisher > _publisherMap;
	    
	    typedef std::map< std::string, Publisher::Publisher>::const_iterator pubConstIter;
	    typedef std::map< std::string, Recorder::Recorder>::const_iterator recConstIter;
	    
	    
	    void rosLoop();
	    void startRosLoop();
	    void stopRosLoop();
	    void registerDefaultConverter();
	    void registerGroup(Sinfonia::Converter::Converter converter, Sinfonia::Publisher::Publisher publisher, Sinfonia::Recorder::Recorder recorder);
	    void registerConverter(Sinfonia::Converter::Converter& converter);
	    void registerPublisher(const std::string& converterName, Sinfonia::Publisher::Publisher& publisher);
	    void registerRecorder(const std::string& converterName, Sinfonia::Recorder::Recorder& recorder, float frequency);	   
	    
    };
}

#endif
