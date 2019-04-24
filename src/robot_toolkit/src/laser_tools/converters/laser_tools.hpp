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


#ifndef LASER_TOOLS_CONVERTER_HPP
#define LASER_TOOLS_CONVERTER_HPP

#include <boost/foreach.hpp>

#include "../../converters/converter_base.hpp"
#include "robot_toolkit/message_actions.h"


#include <sensor_msgs/LaserScan.h>

namespace Sinfonia
{
    namespace Converter
    {

	class LaserToolsConverter : public BaseConverter<LaserToolsConverter>
	{

	    typedef boost::function<void(sensor_msgs::LaserScan&)> CallbackT;

	    public:
		LaserToolsConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session );

		void registerCallback( MessageAction::MessageAction action, CallbackT callback );

		void callAll( const std::vector<MessageAction::MessageAction>& actions );

		void reset( );

	    private:
		std::vector<float> fromAnyValueToFloatVector(qi::AnyValue& value, std::vector<float>& result);
		qi::AnyObject _pMemory;
		std::map<MessageAction::MessageAction, CallbackT> _callbacks;
		sensor_msgs::LaserScan _message;
	};

    } 
} 

#endif
