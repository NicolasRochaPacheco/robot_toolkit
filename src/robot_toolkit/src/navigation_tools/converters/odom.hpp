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


#ifndef ODOM_CONVERTER_HPP
#define ODOM_CONVERTER_HPP

#include "../../converters/converter_base.hpp"
#include <naoqi_driver/message_actions.h>



namespace Sinfonia
{
    namespace Converter
    {

	class OdomConverter : public BaseConverter<OdomConverter>
	{
	    typedef boost::function<void(nav_msgs::Odometry&)> callbackT;

	    public:
		OdomConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session );
		void registerCallback( message_actions::MessageAction action, Callback_t callback );
		void callAll( const std::vector<message_actions::MessageAction>& actions );
		void reset( );

	    private:
		qi::AnyObject _pMotion;
		std::map<message_actions::MessageAction, callbackT> _callbacks;
		nav_msgs::Odometry _msg;
	};

    }
}

#endif
