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


#ifndef CONVERTER_HPP
#define CONVERTER_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <robot_toolkit/message_actions.h>

namespace Sinfonia
{
    namespace Converter
    {
	class Converter
	{
	    public:
		template<typename T>
		Converter( T converter ):
		converterPtr(boost::make_shared<ConverterModel<T> >(converter))
		{
		    
		}
		std::string name() const
		{
		    return converterPtr->name();
		}
		float frequency() const
		{
		    return converterPtr->frequency();
		}

		void reset()
		{
		    converterPtr->reset();
		}

		void callAll( const std::vector<Sinfonia::MessageAction::MessageAction>& actions )
		{
		    if ( actions.size() > 0 )
		    {
			converterPtr->callAll(actions);
		    }

		    ros::Time after = ros::Time::now();
		    lapse_time = after - before;
		    before = after;
		}

		ros::Duration lapseTime() const
		{
		    return lapse_time;
		}

		friend bool operator==( const Converter& lhs, const Converter& rhs )
		{
		    if ( lhs.name() == rhs.name() )
			return true;
		    return false;
		}
		
	    private:
		ros::Time before;
		ros::Duration lapse_time;
		struct ConverterConcept
		{
		    virtual ~ConverterConcept(){}
		    virtual std::string name() const = 0;
		    virtual float frequency() const = 0;
		    virtual void reset() = 0;
		    virtual void callAll( const std::vector<Sinfonia::MessageAction::MessageAction>& actions ) = 0;
		};

		template<typename T>
		struct ConverterModel : public ConverterConcept
		{
		    ConverterModel( const T& other ):
		    converter_( other )
		    {}

		    std::string name() const
		    {
			return converter_->name();
		    }

		    float frequency() const
		    {
			return converter_->frequency();
		    }
		    
		    void reset()
		    {
			converter_->reset();
		    }

		    void callAll( const std::vector<Sinfonia::MessageAction::MessageAction>& actions )
		    {
			converter_->callAll( actions );
		    }

		    T converter_;
		};

		boost::shared_ptr<ConverterConcept> converterPtr;    
	};
    }
}
#endif