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



#include "robot_toolkit/robot_toolkit.hpp"
#include "robot_toolkit/message_actions.h"

#include "robot_toolkit/ros_environment.hpp"



#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace Sinfonia
{
    RobotToolkit::RobotToolkit(qi::SessionPtr session, const std::string& prefix)
    {
	if(prefix == "")
	{
	    std::cout << "Error driver prefix must not be empty" << std::endl;
	    throw new ros::Exception("Error driver prefix must not be empty");
	}
	else
	{
	    Sinfonia::RosEnvironment::setPrefix(prefix);
	}
	_sessionPtr = session;
	_isRosLoopEnabled = true;
	
    }

    RobotToolkit::~RobotToolkit()
    {
	std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << "robot_toolkit is shutting down.." << std::endl;
	if(_nodeHandlerPtr)
	{
	    _nodeHandlerPtr->shutdown();
	    ros::shutdown();
	}
    }
    
    std::string RobotToolkit::_whoWillWin()
    {
	return "SinfonIA SSPL Robocup Team";
    }
    
    void RobotToolkit::init()
    {
	ros::Time::init();
	registerDefaultConverter();
	registerDefaultSubscriber();
	//startRosLoop();
    }
    
    void RobotToolkit::rosLoop()
    {
	static std::vector<MessageAction::MessageAction> actions;
	while(_isRosLoopEnabled)
	{
	    actions.clear();
	    {
		boost::mutex::scoped_lock lock( _mutexConvertersQueue );
		if(!_convertersQueue.empty())
		{
		    size_t converterIndex = _convertersQueue.top()._converterIndex;
		    Converter::Converter& converter = _converters[converterIndex];
		    ros::Time schedule = _convertersQueue.top()._schedule;
		    PublisherConstIterator publisherIterator = _publisherMap.find( converter.name() );
		    if ( _publishEnabled &&  publisherIterator != _publisherMap.end() && publisherIterator->second.isSubscribed() )
		    {
			actions.push_back(MessageAction::PUBLISH);
		    }
		    
		    if ( actions.size()>0 )
		    {
			converter.callAll( actions );
		    }
		    
		    ros::Duration d( schedule - ros::Time::now() );
		    if ( d > ros::Duration(0))
		    {
			d.sleep();
		    }
		    
		    _convertersQueue.pop();
		    if ( converter.getFrequency() != 0 )
		    {
			_convertersQueue.push(Helpers::ScheduledConverter(schedule + ros::Duration(1.0f / converter.getFrequency()), converterIndex));
		    }	
		}
		else
		{
		    ros::Duration(1).sleep();
		}
	    }
	    if ( _publishEnabled )
	    {
		ros::spinOnce();
	    }
	}
    }
    
    void RobotToolkit::startRosLoop()
    {
	if (_mainThread.get_id() ==  boost::thread::id())
	    _mainThread = boost::thread( &RobotToolkit::rosLoop, this );
	_isRosLoopEnabled = true;
    }

    void RobotToolkit::stopRosLoop()
    {
	_isRosLoopEnabled = false;
	if (_mainThread.get_id() !=  boost::thread::id())
	    _mainThread.join();
    }
    
    void RobotToolkit::setMasterURINet(const std::string& uri, const std::string& networkInterface)
    {
	boost::mutex::scoped_lock lock( _mutexConvertersQueue );
	{
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Going to reset the node Handle" << RESETCOLOR << std::endl;
	    _nodeHandlerPtr.reset();
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Nodehandle reset " << std::endl;
	    Sinfonia::RosEnvironment::setMasterURI( uri, networkInterface );
	    _nodeHandlerPtr.reset( new ros::NodeHandle("~") );
	}
	if(_converters.empty())
	{
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Goiing to register converters" << RESETCOLOR << std::endl;
	    registerDefaultConverter();
	    registerDefaultSubscriber();

	}
	else
	{
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "NOT going to re-register the converters" << std::endl;
	    typedef std::map< std::string, Publisher::Publisher > publisherMap; 	    
	    resetService(* _nodeHandlerPtr);
	}
	startPublishing();
	
    }
    

    void RobotToolkit::startPublishing()
    {
	_publishEnabled = true;
    }
    
    void RobotToolkit::startInitialTopics()
    {
	// Poner aqui el schedule de lso topicos que deben iniar desde el inicio 
	//scheduleConverter("front_camera", 10.0f);  
	startRosLoop();
	std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Robot Toolkit Ready !!!" << std::endl;
    }


    void RobotToolkit::stopService()
    {
	stopRosLoop();
	_converters.clear();
	_subscribers.clear();
    }

    void RobotToolkit::registerDefaultConverter()
    {
	_tf2Buffer.reset<tf2_ros::Buffer>( new tf2_ros::Buffer() );
	_tf2Buffer->setUsingDedicatedThread(true);

	boost::shared_ptr<Publisher::TfPublisher> tfPublisher = boost::make_shared<Publisher::TfPublisher>();
	boost::shared_ptr<Converter::TfConverter> tfConverter = boost::make_shared<Converter::TfConverter>( "tf", 50, _tf2Buffer, _sessionPtr );
	tfConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::TfPublisher::publish, tfPublisher, _1) );
	registerGroup( tfConverter, tfPublisher);
	
	
	boost::shared_ptr<Publisher::OdomPublisher > odomPublisher = boost::make_shared<Publisher::OdomPublisher>();
	boost::shared_ptr<Converter::OdomConverter> odomConverter = boost::make_shared<Converter::OdomConverter>( "odom", 10, _sessionPtr );
	odomConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::OdomPublisher::publish, odomPublisher, _1) );
	registerGroup( odomConverter, odomPublisher);
	
	boost::shared_ptr<Publisher::LaserPublisher> laserPublisher = boost::make_shared<Publisher::LaserPublisher>();
	boost::shared_ptr<Converter::LaserConverter> laserConverter = boost::make_shared<Converter::LaserConverter>( "laser", 10, _sessionPtr );
	laserConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::LaserPublisher::publish, laserPublisher, _1) );
	registerGroup( laserConverter, laserPublisher);
	
	boost::shared_ptr<Publisher::CameraPublisher> frontCameraPublisher = boost::make_shared<Publisher::CameraPublisher>("camera/front/image_raw");
	boost::shared_ptr<Converter::CameraConverter> frontCameraConverter = boost::make_shared<Converter::CameraConverter>( "front_camera", 10, _sessionPtr, Helpers::VisionHelpers::kTopCamera, Helpers::VisionHelpers::kQVGA, Helpers::VisionHelpers::kRGBColorSpace);
	frontCameraConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::CameraPublisher::publish, frontCameraPublisher, _1, _2) );
	registerGroup( frontCameraConverter, frontCameraPublisher);
	
	boost::shared_ptr<Publisher::CameraPublisher> bottomCameraPublisher = boost::make_shared<Publisher::CameraPublisher>("camera/bottom/image_raw");
	boost::shared_ptr<Converter::CameraConverter> bottomCameraConverter = boost::make_shared<Converter::CameraConverter>( "bottom_camera", 10, _sessionPtr, Helpers::VisionHelpers::kBottomCamera, Helpers::VisionHelpers::kQVGA, Helpers::VisionHelpers::kRGBColorSpace);
	bottomCameraConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::CameraPublisher::publish, bottomCameraPublisher, _1, _2) );
	registerGroup( bottomCameraConverter, bottomCameraPublisher);
	
	boost::shared_ptr<Publisher::CameraPublisher> depthCameraPublisher = boost::make_shared<Publisher::CameraPublisher>("camera/depth/image_raw");
	boost::shared_ptr<Converter::CameraConverter> depthCameraConverter = boost::make_shared<Converter::CameraConverter>( "depth_camera", 10, _sessionPtr, Helpers::VisionHelpers::kDepthCamera, Helpers::VisionHelpers::kQVGA, Helpers::VisionHelpers::kRawDepthColorSpace);
	depthCameraConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::CameraPublisher::publish, depthCameraPublisher, _1, _2) );
	registerGroup( depthCameraConverter, depthCameraPublisher);

	printRegisteredConverters();
	
    }

    void RobotToolkit::registerGroup(Converter::Converter converter, Publisher::Publisher publisher)
    {
	registerConverter(converter);
	registerPublisher(converter.name(), publisher);
    }
    
    void RobotToolkit::registerConverter(Converter::Converter& converter)
    {
	boost::mutex::scoped_lock lock( _mutexConvertersQueue );
	int convIndex = _converters.size();
	_converters.push_back( converter );
	converter.reset();
	
    }

    void RobotToolkit::registerPublisher(const std::string& converterName, Publisher::Publisher& publisher)
    {
	_publisherMap.insert( std::map<std::string, Publisher::Publisher>::value_type(converterName, publisher) );
    }
    
    void RobotToolkit::printRegisteredConverters()
    {
	for( int i=0; i<_converters.size(); i++ )
	{
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Registered /" << _converters[i].name() << " converter. " << std::endl;
	}
    }

    void RobotToolkit::registerDefaultSubscriber()
    {
	if (!_subscribers.empty())
	    return;
	registerSubscriber(boost::make_shared<Subscriber::CmdVelSubscriber>("cmd_vel", "/cmd_vel", _sessionPtr));
    }
    
    void RobotToolkit::registerSubscriber(Subscriber::Subscriber subscriber)
    {
	std::vector<Subscriber::Subscriber>::iterator it;
	it = std::find( _subscribers.begin(), _subscribers.end(), subscriber );
	size_t subIndex = 0;

	if (it == _subscribers.end() )
	{
	    subIndex = _subscribers.size();
	    _subscribers.push_back( subscriber );
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "Registered /" << subscriber.name() << " subscriber."<< std::endl;
	}

	else
	{
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "] " << "re-initialized existing subscriber:\t" << it->name() << std::endl;
	}
    }
    
    void RobotToolkit::scheduleConverter(std::string converterName, float converterFrequency)
    {
	boost::mutex::scoped_lock lock( _mutexConvertersQueue );
	std::priority_queue<Helpers::ScheduledConverter> auxiliarQueue = _convertersQueue;
	bool exist = false;
	for( int i=0; i<_convertersQueue.size(); i++)
	{
	    int converterIndex = auxiliarQueue.top()._converterIndex;
	    if(_converters[converterIndex].name() == converterName)
	    {
		_converters[converterIndex].setFrequency(converterFrequency);
		exist = true;
		break;
	    }
	    auxiliarQueue.pop();
	    
	}
	if (!exist)
	{
	    for( int i=0; i<_converters.size(); i++ )
	    {
		if(_converters[i].name() == converterName)
		{
		    _converters[i].setFrequency(converterFrequency);
		    _convertersQueue.push(Helpers::ScheduledConverter(ros::Time::now(), i));
		}
	    }
	    typedef std::map< std::string, Publisher::Publisher > publisherMap;
	    for_each( publisherMap::value_type &pub, _publisherMap )
	    {
		if (pub.first.c_str() == converterName)
		{
		    pub.second.reset(*_nodeHandlerPtr);
		}
	    }
	}	
    }
  
    void RobotToolkit::unscheduleConverter(std::string converterName)
    {
	boost::mutex::scoped_lock lock( _mutexConvertersQueue );
	std::vector< Helpers::ScheduledConverter > auxiliarQueue;
	size_t converterIndex;
	typedef std::map< std::string, Publisher::Publisher > publisherMap;
	int queueSize = _convertersQueue.size();
	for( int i=0; i<queueSize; i++)
	{
	    converterIndex = _convertersQueue.top()._converterIndex;
	    Converter::Converter& converter = _converters[converterIndex];
	    if(converter.name() != converterName)
	    {
		auxiliarQueue.push_back(_convertersQueue.top());
	    }
	    _convertersQueue.pop();
	}
	for( int i = 0; i < auxiliarQueue.size(); i++)
	{
	    _convertersQueue.push(auxiliarQueue[i]);
	}
	for_each( publisherMap::value_type &pub, _publisherMap )
	{
	    if (pub.first.c_str() == converterName)
	    {
		pub.second.shutdown();
	    }
	}
    }

    void RobotToolkit::startSubscriber(std::string subscriberName)
    {
	for_each( Subscriber::Subscriber& sub, _subscribers )
	{
	    if(sub.name() == subscriberName)
	    {
		sub.reset( *_nodeHandlerPtr );
	    }
	}
    }
    
    void RobotToolkit::stopSubscriber(std::string subscriberName)
    {
	for_each( Subscriber::Subscriber& sub, _subscribers )
	{
	    if(sub.name() == subscriberName)
	    {
		sub.shutdown();
	    }
	}
    }
    
    void RobotToolkit::resetService(ros::NodeHandle& nodeHandle)
    {
	_navigationToolsService = nodeHandle.advertiseService("/robot_tookit/navigation_tools_service" , &RobotToolkit::navigationToolsCallback, this);
	_visionToolsService = nodeHandle.advertiseService("/robot_tookit/vision_tools_service" , &RobotToolkit::visionToolsCallback, this);
    }
    
    bool RobotToolkit::navigationToolsCallback( robot_toolkit_msgs::navigation_tools_srv::Request& request, robot_toolkit_msgs::navigation_tools_srv::Response& response )
    {
	std::string responseMessage;
	if( request.data.command == "enable_all" )
	{
	    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "]" << " Starting Navigation Tools " << RESETCOLOR  << std::endl;
	    scheduleConverter("tf", 50.0f);
	    scheduleConverter("odom", 10.0f);
	    scheduleConverter("laser", 10.0f);
	    startSubscriber("cmd_vel");
	    responseMessage = "Functionalities started: tf@50Hz, odom@10Hz, laser@10Hz, cmd_vel";
	    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
	}
	else if( request.data.command == "disable_all" )
	{
	    std::cout << BOLDBLUE << "[" << ros::Time::now().toSec() << "]" << " Stopping Navigation Tools " << RESETCOLOR  << std::endl;
	    unscheduleConverter("tf");
	    unscheduleConverter("odom");
	    unscheduleConverter("laser");
	    stopSubscriber("cmd_vel");
	    responseMessage = "Functionalities stopped: tf, odom, laser, cmd_vel";
	    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
	}
	else if( request.data.command == "custom" )
	{
	    std::cout << BOLDMAGENTA << "[" << ros::Time::now().toSec() << "]" << " Setting up navigation_tools " << RESETCOLOR  << std::endl;
	    std::string startedFunctionalities = "Functionalities started: ";
	    std::string stoppedFunctionalities = "Functionalities stopped: ";
	    responseMessage = "topics";
	    if( request.data.tf_enable )
	    {
		scheduleConverter("tf", request.data.tf_frequency);
		startedFunctionalities += "tf@" + boost::lexical_cast<std::string>(request.data.tf_frequency) + "Hz, ";
	    }
	    else 
	    {
		unscheduleConverter("tf");
		stoppedFunctionalities += "tf, ";
	    }
	    
	    if( request.data.odom_enable )
	    {
		startedFunctionalities += "odom@" + boost::lexical_cast<std::string>(request.data.odom_frequency) + "Hz, ";
		scheduleConverter("odom", request.data.odom_frequency);
	    }
	    else
	    {
		unscheduleConverter("odom");
		stoppedFunctionalities += "odom, ";
	    }
	    
	    if( request.data.laser_enable )
	    {
		startedFunctionalities += "laser@" + boost::lexical_cast<std::string>(request.data.laser_frequency) + "Hz, ";
		scheduleConverter("laser", request.data.laser_frequency);
	    }
	    else 
	    {
		unscheduleConverter("laser");
		stoppedFunctionalities += "laser, ";
	    }
	    
	    if( request.data.cmd_vel_enable )
	    {
		startSubscriber("cmd_vel");
		startedFunctionalities += "cmd_vel, ";
	    }
	    else
	    {
		stopSubscriber("cmd_vel");
		stoppedFunctionalities += "cmd_vel, ";
	    }
	    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << startedFunctionalities << RESETCOLOR  << std::endl;
	    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << stoppedFunctionalities << RESETCOLOR  << std::endl;
	    responseMessage = startedFunctionalities + stoppedFunctionalities;
	}
	else
	{
	    responseMessage = "ERROR: unkown command in navigation_tools service";
	    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
	}
	response.result =  responseMessage;
	return true;
    }
    
    bool RobotToolkit::visionToolsCallback(robot_toolkit_msgs::vision_tools_srv::Request& request, robot_toolkit_msgs::vision_tools_srv::Response& response)
    {
	std::string responseMessage;
	robot_toolkit_msgs::camera_parameters_msg currentParamsMessage;
	if( request.data.camera_name != "front_camera" && request.data.camera_name != "bottom_camera" && request.data.camera_name != "depth_camera" ) 
	{
	    responseMessage = "ERROR: unknown camera name, possible values are: front_camera, bottom_camera, depth_camera";
	    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
	}
	else
	{
	    if( request.data.command == "enable" )
	    {
		int converterIndex = getConverterIndex(request.data.camera_name);
		if( converterIndex != -1 )
		{
		    std::vector<int> config;
		    config.push_back(Helpers::VisionHelpers::kQVGA);
		    config.push_back(10);
		    if( request.data.camera_name == "depth_camera" )
		    {
			config.push_back(Helpers::VisionHelpers::kRawDepthColorSpace);
		    }
		    else
		    {
			config.push_back(Helpers::VisionHelpers::kRGBColorSpace);
		    }
		    _converters[converterIndex].setConfig(config);
		    _converters[converterIndex].reset();
		    if( request.data.camera_name != "depth_camera" )
		    {
			currentParamsMessage = toCameraParametersMsg(_converters[converterIndex].setAllParametersToDefault());
		    }
		    scheduleConverter(request.data.camera_name, 10.0f); 
		    responseMessage = "Starting: " + request.data.camera_name + " with default parameters";
		    std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "]" << " Starting: " << request.data.camera_name << " with default parameters" << RESETCOLOR  << std::endl;		    
		}
		else
		{
		    responseMessage = "ERROR: converter missing ";
		    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
		}
	    }   
	    else if( request.data.command == "disable" )
	    {
		int converterIndex = getConverterIndex(request.data.camera_name);
		if( converterIndex != -1 )
		{	    
		    unscheduleConverter(request.data.camera_name);   
		    responseMessage = "Shutting down: " + request.data.camera_name;
		    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "]" << " Shutting down: " << request.data.camera_name << RESETCOLOR  << std::endl;
		}
		else
		{
		    responseMessage = "ERROR: converter missing ";
		    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
		    
		}
	    }   
	    else if( request.data.command == "custom" )
	    {
		int converterIndex = getConverterIndex(request.data.camera_name);
		if( converterIndex != -1 )
		{
		    std::vector<int> config;
		    bool configOk = true;
		    if( request.data.camera_name != "depth_camera" )
		    {
			if(!(request.data.resolution <= Helpers::VisionHelpers::k16VGA || request.data.resolution == Helpers::VisionHelpers::kQQQVGA || request.data.resolution == Helpers::VisionHelpers::kQQQQVGA ))
			{
			    responseMessage = "ERROR: Bad resolution configuration for camera: " + request.data.camera_name;
			    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
			    configOk = false;
			}
			else if( request.data.frame_rate < 1 || request.data.frame_rate > 30 || ((request.data.resolution == Helpers::VisionHelpers::k4VGA) && (request.data.frame_rate != 1)) || 
			    ((request.data.resolution == Helpers::VisionHelpers::k16VGA) && (request.data.frame_rate != 1)))
			{
			    responseMessage = "ERROR: Bad frame rate configuration for camera: " + request.data.camera_name;
			    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
			    configOk = false;
			}
			else if( request.data.color_space > Helpers::VisionHelpers::kHSMixedColorSpace)
			{
			    responseMessage = "ERROR: Bad color space configuration for camera: " + request.data.camera_name;
			    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
			    configOk = false;
			}
		    }
		    else 
		    {
			if(!(request.data.resolution <= Helpers::VisionHelpers::kQVGA || request.data.resolution == Helpers::VisionHelpers::kQQQVGA || request.data.resolution == Helpers::VisionHelpers::kQQQQVGA ))
			{
			    responseMessage = "ERROR: Bad resolution configuration for camera: " + request.data.camera_name;
			    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
			    configOk = false;
			}
			else if( request.data.frame_rate < 1 || request.data.frame_rate > 20 )
			{
			    responseMessage = "ERROR: Bad frame rate configuration for camera: " + request.data.camera_name;
			    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
			    configOk = false;
			}
			else if( !( request.data.color_space == Helpers::VisionHelpers::kYuvColorSpace ||  request.data.color_space == Helpers::VisionHelpers::kRGBColorSpace || request.data.color_space == Helpers::VisionHelpers::kDepthColorSpace ||
			    request.data.color_space == Helpers::VisionHelpers::kXYZColorSpace || request.data.color_space == Helpers::VisionHelpers::kDistanceColorSpace || request.data.color_space == Helpers::VisionHelpers::kRawDepthColorSpace ) )
			{
			    responseMessage = "ERROR: Bad color space configuration for camera: " + request.data.camera_name;
			    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
			    configOk = false;
			}
		    }
		    if(configOk)
		    {
			std::vector<int> config;
			config.push_back(request.data.resolution);
			config.push_back(request.data.frame_rate);
			config.push_back(request.data.color_space);
			_converters[converterIndex].setConfig(config);
			_converters[converterIndex].reset();
			if( request.data.camera_name != "depth_camera" )
			{
			    currentParamsMessage = toCameraParametersMsg(_converters[converterIndex].setAllParametersToDefault());
			}
			scheduleConverter(request.data.camera_name, request.data.frame_rate);
			responseMessage = "Starting: " + request.data.camera_name + " with custom configuration and parameters";
			std::cout << BOLDGREEN << "[" << ros::Time::now().toSec() << "]" << responseMessage << RESETCOLOR  << std::endl;
			
		    }
		}
		else
		{
		    responseMessage = "ERROR: converter missing ";
		    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
		    
		}
	    }   
	    else if( request.data.command == "set_parameters" && request.data.camera_name != "depth_camera")
	    {
		    int converterIndex = getConverterIndex(request.data.camera_name);
		    if( converterIndex != -1 )
		    {
			currentParamsMessage = toCameraParametersMsg(_converters[converterIndex].setParameters(toVector(request.data.camera_parameters)));  
			responseMessage = "Setting Parameters of " + request.data.camera_name;
			std::cout << BOLDMAGENTA << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
		    }
		    else
		    {
			responseMessage = "ERROR: converter missing ";
			std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
			
		    }
	    }
	    else if( request.data.command == "get_parameters" && request.data.camera_name != "depth_camera")
	    {
		int converterIndex = getConverterIndex(request.data.camera_name);
		if( converterIndex != -1 )
		{
		    currentParamsMessage = toCameraParametersMsg(_converters[converterIndex].getParameters());  
		    responseMessage = "Getting Parameters of " + request.data.camera_name;
		    std::cout << BOLDMAGENTA << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
		}
		else
		{
		    responseMessage = "ERROR: converter missing ";
		    std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
		}
	    }
	    else
	    {
		responseMessage = "ERROR: unkown command in vision_tools service";
		std::cout << BOLDRED << "[" << ros::Time::now().toSec() << "] " << responseMessage << RESETCOLOR  << std::endl;
	    }
	}
	response.result = responseMessage;
	response.camera_parameters = currentParamsMessage;
	return true;
    }

    int RobotToolkit::getConverterIndex(std::string name)
    {
	for( int i=0; i<_converters.size(); i++ )
	{
	    if(_converters[i].name() == name)
	    {
		return i;
	    }
	}
	return -1;
    }
    
    robot_toolkit_msgs::camera_parameters_msg RobotToolkit::toCameraParametersMsg(std::vector< int > params)
    {
	robot_toolkit_msgs::camera_parameters_msg result;
	result.brightness = params[0];
	result.contrast = params[1];
	result.saturation = params[2];
	result.hue = params[3];
	result.horizontal_flip = params[4];
	result.vertical_flip = params[5];
	result.auto_exposition = params[6];
	result.auto_white_balance = params[7];
	result.auto_gain = params[8];
	result.gain = params[9];
	result.exposure = params[10];
	result.reset_camera_registers = params[11];
	result.blc_red_value = params[12];
	result.blc_green_value = params[13];
	result.blc_blue_value = params[14];
	result.resolution = params[15];
	result.fps = params[16];
	result.average_luminance = params[17];
	result.auto_focus = params[18];
	return result;
    }

    std::vector< int > RobotToolkit::toVector(robot_toolkit_msgs::camera_parameters_msg params)
    {
	std::vector<int>  result; 
	result.push_back(params.brightness);
	result.push_back(params.contrast);
	result.push_back(params.saturation);
	result.push_back(params.hue);
	result.push_back(params.horizontal_flip);
	result.push_back(params.vertical_flip);
	result.push_back(params.auto_exposition);
	result.push_back(params.auto_white_balance);
	result.push_back(params.auto_gain);
	result.push_back(params.gain);
	result.push_back(params.exposure);
	result.push_back(params.reset_camera_registers);
	result.push_back(params.blc_red_value);
	result.push_back(params.blc_green_value);
	result.push_back(params.blc_blue_value);
	result.push_back(params.resolution);
	result.push_back(params.fps);
	result.push_back(params.average_luminance);
	result.push_back(params.auto_focus);
	return result;
    }

    
    QI_REGISTER_OBJECT( RobotToolkit, _whoWillWin, setMasterURINet, startPublishing);
}
