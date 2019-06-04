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
#include <string>

#include <queue>

#include <tf2_ros/buffer.h>

#include "ros/ros.h"

#include "robot_toolkit_msgs/navigation_tools_msg.h"
#include "robot_toolkit_msgs/navigation_tools_srv.h"

#include "robot_toolkit_msgs/camera_parameters_msg.h"
#include "robot_toolkit_msgs/vision_tools_msg.h"
#include "robot_toolkit_msgs/vision_tools_srv.h"
#include "robot_toolkit_msgs/audio_tools_msg.h"
#include "robot_toolkit_msgs/audio_tools_srv.h"
#include "robot_toolkit_msgs/speech_parameters_msg.h"
#include "robot_toolkit_msgs/motion_tools_msg.h"
#include "robot_toolkit_msgs/motion_tools_srv.h"

#include "robot_toolkit/ros_environment.hpp"
#include "robot_toolkit/converter/converter.hpp"
#include "robot_toolkit/publisher/publisher.hpp"
#include "robot_toolkit/subscriber/subscriber.hpp"
#include "robot_toolkit/event/event.hpp"

#include "robot_toolkit/helpers/scheduled_conveter.hpp"

#include "robot_toolkit/navigation_tools/tf/tf_publisher.hpp"
#include "robot_toolkit/navigation_tools/odom/odom_publisher.hpp"
#include "robot_toolkit/navigation_tools/laser/laser_publisher.hpp"
#include "robot_toolkit/navigation_tools/move_to/move_to.hpp"
#include "robot_toolkit/navigation_tools/tf/tf_converter.hpp"
#include "robot_toolkit/navigation_tools/odom/odom_converter.hpp"
#include "robot_toolkit//navigation_tools/laser/laser_converter.hpp"
#include "robot_toolkit//navigation_tools/laser/depth_to_laser_converter.hpp"
#include "robot_toolkit/navigation_tools/cmd_vel/cmd_vel_subscriber.hpp"

#include "robot_toolkit/vision_tools/camera_converter.hpp"
#include "robot_toolkit/vision_tools/camera_publisher.hpp"

#include "robot_toolkit/audio_tools/mic/mic_event.hpp"
#include "robot_toolkit/audio_tools/speech/speech_subscriber.hpp"
#include "robot_toolkit/audio_tools/mic/mic_localization_event.hpp"

#include "robot_toolkit/motion_tools/animation_subscriber.hpp"
#include "robot_toolkit/motion_tools/set_angles_subscriber.hpp"



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
	    
	    void startInitialTopics();
	    
	    bool navigationToolsCallback(robot_toolkit_msgs::navigation_tools_srv::Request& request, robot_toolkit_msgs::navigation_tools_srv::Response& response);
	    bool visionToolsCallback(robot_toolkit_msgs::vision_tools_srv::Request& request, robot_toolkit_msgs::vision_tools_srv::Response& response);
	    bool audioToolsCallback(robot_toolkit_msgs::audio_tools_srv::Request& request, robot_toolkit_msgs::audio_tools_srv::Response& response );
	    bool motionToolsCallback(robot_toolkit_msgs::motion_tools_srv::Request& request, robot_toolkit_msgs::motion_tools_srv::Response& response);
	    
	private:
	    
	    boost::thread _mainThread;
	    
	    bool _logEnabled;
	    bool _recordEnabled;
	    bool _publishEnabled;
	    bool _isRosLoopEnabled;
	    
	    
	    boost::shared_ptr<tf2_ros::Buffer> _tf2Buffer;
	    
	    
	    boost::scoped_ptr<ros::NodeHandle> _nodeHandlerPtr;
	    
	    
	    boost::mutex _mutexRecorders;
	    boost::mutex _mutexConvertersQueue;
	    
	    qi::SessionPtr _sessionPtr;
	    
	    
	    std::vector< Converter::Converter > _converters;
	    std::vector< Subscriber::Subscriber > _subscribers;
	    
	    std::priority_queue<Helpers::ScheduledConverter> _convertersQueue;
	    
	    ros::ServiceServer _navigationToolsService;
	    ros::ServiceServer _visionToolsService;
	    ros::ServiceServer _audioToolsService;
	    ros::ServiceServer _motionToolsService;
	    
	    std::map< std::string, Publisher::Publisher > _publisherMap;
	    std::map< std::string, Event::Event> _eventMap;
	    
	    
	    typedef std::map< std::string, Publisher::Publisher>::const_iterator PublisherConstIterator;   
	    
	    void rosLoop();
	    void startRosLoop();
	    void stopRosLoop();
	    void registerDefaultConverter();
	    void registerGroup(Sinfonia::Converter::Converter converter, Sinfonia::Publisher::Publisher publisher);
	    void registerConverter(Sinfonia::Converter::Converter& converter);
	    void printRegisteredConverters();
	    void registerPublisher(const std::string& converterName, Sinfonia::Publisher::Publisher& publisher);
	    void registerDefaultSubscriber();
	    void registerSubscriber(Sinfonia::Subscriber::Subscriber subscriber);
	    void resetService(ros::NodeHandle& nodeHandle);
	    void scheduleConverter(std::string converterName, float converterFrequency);
	    void unscheduleConverter(std::string converterName);
	    void startSubscriber(std::string subscriberName);
	    void stopSubscriber(std::string subscriberName);
	    int getConverterIndex(std::string name);
	    void insertEventConverter( const std::string& key, Event::Event event);	    
	    robot_toolkit_msgs::camera_parameters_msg toCameraParametersMsg(std::vector<float> params);
	    std::vector<float> toVector(robot_toolkit_msgs::camera_parameters_msg params);
	    int getSubscriberIndex(std::string name);
	    robot_toolkit_msgs::speech_parameters_msg toSpeechParameters(std::vector<float> params);
    };
}

#endif
