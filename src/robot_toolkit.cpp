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
	std::cout << "robot_toolkit is shutting down.." << std::endl;
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
	startRosLoop();
    }
    
    void RobotToolkit::rosLoop()
    {
	static std::vector<MessageAction::MessageAction> actions;
	ROS_INFO("Ready to initTF.");
	while(_isRosLoopEnabled)
	{
	    actions.clear();
	    {
		boost::mutex::scoped_lock lock( _mutexConvertersQueue );
		if(!_convertersQueue.empty())
		{
		    size_t convIndex = _convertersQueue.top().conv_index_;
		    Converter::Converter& conv = _converters[convIndex];
		    ros::Time schedule = _convertersQueue.top().schedule_;

		    pubConstIter pubIt = _publisherMap.find( conv.name() );
		    if ( _publishEnabled &&  pubIt != _publisherMap.end() && pubIt->second.isSubscribed() )
		    {
			actions.push_back(MessageAction::PUBLISH);
		    }

		    if (actions.size() >0)
		    {
			conv.callAll( actions );
		    }

		    ros::Duration d( schedule - ros::Time::now() );
		    if ( d > ros::Duration(0))
		    {
			d.sleep();
		    }

		    _convertersQueue.pop();
		    if ( conv.frequency() != 0 )
		    {
			_convertersQueue.push(ScheduledConverter(schedule + ros::Duration(1.0f / conv.frequency()), convIndex));
		    }	
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
	    _nodeHandlerPtr.reset();
	    std::cout << "nodehandle reset " << std::endl;
	    Sinfonia::RosEnvironment::setMasterURI( uri, networkInterface );
	    _nodeHandlerPtr.reset( new ros::NodeHandle("~") );
	}
	if(_converters.empty())
	{
	    std::cout << BOLDRED << "going to register converters" << RESETCOLOR << std::endl;
	    registerDefaultConverter();
	    registerDefaultSubscriber();

	}
	else
	{
	    std::cout << "NOT going to re-register the converters" << std::endl;
	    typedef std::map< std::string, Publisher::Publisher > publisherMap;
	    for_each( publisherMap::value_type &pub, _publisherMap )
	    {
		pub.second.reset(*_nodeHandlerPtr);
	    }
	    
	    for_each( Subscriber::Subscriber& sub, _subscribers )
	    {
		std::cout << "resetting subscriber " << sub.name() << std::endl;
		sub.reset( *_nodeHandlerPtr );
	    }
	    /*for_each( Service::Service& srv, _services )
	    {
		std::cout << "resetting service " << srv.name() << std::endl;
		srv.reset( *_nodeHandlerPtr );
	    }**/
	    resetService(* _nodeHandlerPtr);
	}
	startPublishing();
	
    }
    

    void RobotToolkit::startPublishing()
    {
	_publishEnabled = true;
    }


    void RobotToolkit::stopService()
    {
	stopRosLoop();
    }

    void RobotToolkit::registerDefaultConverter()
    {
	
	
	boost::shared_ptr<Publisher::OdomPublisher > odomPublisher = boost::make_shared<Publisher::OdomPublisher>();
	boost::shared_ptr<Converter::OdomConverter> odomConverter = boost::make_shared<Converter::OdomConverter>( "odom", 20, _sessionPtr );
	odomConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::OdomPublisher::publish, odomPublisher, _1) );
	registerGroup( odomConverter, odomPublisher);
	
	boost::shared_ptr<Publisher::LaserPublisher> laserPublisher = boost::make_shared<Publisher::LaserPublisher>();
	boost::shared_ptr<Converter::LaserConverter> laserConverter = boost::make_shared<Converter::LaserConverter>( "laser", 20, _sessionPtr );
	laserConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::LaserPublisher::publish, laserPublisher, _1) );
	registerGroup( laserConverter, laserPublisher);
	
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
	_convertersQueue.push(ScheduledConverter(ros::Time::now(), convIndex));
    }

    void RobotToolkit::registerPublisher(const std::string& converterName, Publisher::Publisher& publisher)
    {
	if (_publishEnabled) 
	{
	    publisher.reset(*_nodeHandlerPtr);
	}
	
	_publisherMap.insert( std::map<std::string, Publisher::Publisher>::value_type(converterName, publisher) );
	
    }
    
    void RobotToolkit::registerDefaultSubscriber()
    {
	std::cout << "registered DefaulerDefault 1"<< std::endl;
	if (!_subscribers.empty())
	    return;
	std::cout << "registered DefaulerDefault 2"<< std::endl;
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
	    std::cout << "registered subscriber:\t" << subscriber.name() << std::endl;
	}

	else
	{
	    std::cout << "re-initialized existing subscriber:\t" << it->name() << std::endl;
	}
    }
    
    void RobotToolkit::resetService(ros::NodeHandle& nodeHandle)
    {
	_serviceTf = nodeHandle.advertiseService("initTf" , &RobotToolkit::callbackTf, this);
    }
    
    bool RobotToolkit::callbackTf(robot_toolkit_msgs::InitTf::Request& req, robot_toolkit_msgs::InitTf::Response& res)
    {
	if(req.data == "on")
	{    
	    ROS_INFO("Starting tf! ");
	    _tf2Buffer.reset<tf2_ros::Buffer>( new tf2_ros::Buffer() );
	    _tf2Buffer->setUsingDedicatedThread(true);

	    boost::shared_ptr<Publisher::TfPublisher> tfPublisher = boost::make_shared<Publisher::TfPublisher>();
	    boost::shared_ptr<Converter::TfConverter> tfConverter = boost::make_shared<Converter::TfConverter>( "tf", 20, _tf2Buffer, _sessionPtr );
	    tfConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::TfPublisher::publish, tfPublisher, _1) );
	    registerGroup( tfConverter, tfPublisher);
	}
	else 
	{
	    ROS_INFO("Shutting down tf! ");
	    _convertersQueue = std::priority_queue<ScheduledConverter>();
	}
	return true;
    }


    QI_REGISTER_OBJECT( RobotToolkit, _whoWillWin, setMasterURINet, startPublishing);
}
