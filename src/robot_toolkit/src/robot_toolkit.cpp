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

#include "publishers/navigation_tools.hpp"

#include "converters/navigation_tools.hpp"

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
	startRosLoop();
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

	}
	else
	{
	    std::cout << "NOT going to re-register the converters" << std::endl;
	    typedef std::map< std::string, Publisher::Publisher > publisherMap;
	    for_each( publisherMap::value_type &pub, _publisherMap )
	    {
		pub.second.reset(*_nodeHandlerPtr);
	    }
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
	
	_tf2Buffer.reset<tf2_ros::Buffer>( new tf2_ros::Buffer() );
	_tf2Buffer->setUsingDedicatedThread(true);
	
	boost::shared_ptr<Publisher::NavigationToolsPublisher> navigationToolsPublisher = boost::make_shared<Publisher::NavigationToolsPublisher>();
	boost::shared_ptr<Converter::NavigationToolsConverter> navigationToolsConverter = boost::make_shared<Converter::NavigationToolsConverter>( "navigation_tools", 50, _tf2Buffer, _sessionPtr );
	navigationToolsConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::NavigationToolsPublisher::publishTF, navigationToolsPublisher, _1) );
	
	registerGroup( navigationToolsConverter, navigationToolsPublisher);
    }

    void RobotToolkit::registerGroup(Converter::Converter converter, Publisher::Publisher publisher)
    {
	registerConverter(converter);
	registerPublisher(converter.name(), publisher);
	//registerRecorder(converter.name(), recorder, converter.frequency());
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

    QI_REGISTER_OBJECT( RobotToolkit, _whoWillWin, setMasterURINet, startPublishing);
}
