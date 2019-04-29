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
#include "robot_toolkit/subscriber/subscriber.hpp"



#include "../../src/navigation_tools/tf/tf_publisher.hpp"
#include "../../src/navigation_tools/odom/odom_publisher.hpp"
#include "../../src/navigation_tools/laser/laser_publisher.hpp"

#include "../../src/navigation_tools/tf/tf_converter.hpp"
#include "../../src/navigation_tools/odom/odom_converter.hpp"
#include "../../src/navigation_tools/laser/laser_converter.hpp"

#include "../../src/navigation_tools/cmd_vel/cmd_vel_subscriber.hpp"

#include "robot_toolkit_msgs/InitTf.h"

#include <tf2_ros/buffer.h>

#include "ros/ros.h"



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
	    
	    bool initTf();
	    bool callbackTf( robot_toolkit_msgs::InitTf::Request& req, robot_toolkit_msgs::InitTf::Response& res );

	    
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
	    
	    boost::thread _mainThread;
	    
	    bool _logEnabled;
	    bool _recordEnabled;
	    bool _publishEnabled;
	    bool _isRosLoopEnabled;
	    
	    
	    boost::shared_ptr<tf2_ros::Buffer> _tf2Buffer;
	    
	    
	    boost::scoped_ptr<ros::NodeHandle> _nodeHandlerPtr;
	    boost::shared_ptr<Recorder::GlobalRecorder> _recorder;
	    
	    boost::mutex _mutexRecorders;
	    boost::mutex _mutexConvertersQueue;
	    
	    qi::SessionPtr _sessionPtr;
	    
	    
	    std::vector< Converter::Converter > _converters;
	    std::vector< Subscriber::Subscriber > _subscribers;
	    std::priority_queue<ScheduledConverter> _convertersQueue;
	    
	    ros::ServiceServer _serviceTf;
	    
	    
	    
	    std::map< std::string, Recorder::Recorder > _recorderMap;
	    std::map< std::string, Publisher::Publisher > _publisherMap;
	    
	    typedef std::map< std::string, Publisher::Publisher>::const_iterator pubConstIter;
	    typedef std::map< std::string, Recorder::Recorder>::const_iterator recConstIter;
	    
	    
	    
	    void rosLoop();
	    void startRosLoop();
	    void stopRosLoop();
	    void registerDefaultConverter();
	    void registerGroup(Sinfonia::Converter::Converter converter, Sinfonia::Publisher::Publisher publisher);
	    void registerConverter(Sinfonia::Converter::Converter& converter);
	    void registerPublisher(const std::string& converterName, Sinfonia::Publisher::Publisher& publisher);
	    void registerDefaultSubscriber();
	    void registerSubscriber(Sinfonia::Subscriber::Subscriber subscriber);
	    void resetService(ros::NodeHandle& nodeHandle);
    };
}

#endif
