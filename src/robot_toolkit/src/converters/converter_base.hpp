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

#ifndef BASE_CONVERTER_HPP
#define BASE_CONVERTER_HPP

#include "robot_toolkit/tools.hpp"

#include "../helpers/toolkit_helpers.hpp"

#include <qi/session.hpp>
#include <qi/anyobject.hpp>

namespace Sinfonia
{
    namespace Converter
    {
	template<class T>
	class BaseConverter
	{

	    public:
		BaseConverter(const std::string& name, float frequency, qi::SessionPtr session):
		    name_( name ),
		    frequency_( frequency ),
		    robot_(Helpers::Toolkit::getRobot(session)),
		    session_(session),
		    record_enabled_(false)
		{}

		virtual ~BaseConverter(){}

		inline std::string name() const
		{
		    return name_;
		}

		inline float frequency() const
		{
		    return frequency_;
		}

	    protected:
		std::string name_;

		float frequency_;
		const Robot::Robot& robot_;

		qi::SessionPtr session_;

		bool record_enabled_;
	}; 

    }
} 

#endif
