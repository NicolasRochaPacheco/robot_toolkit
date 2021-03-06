/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/


#ifndef NAOQI_ENVIRONMENT_HPP
#define NAOQI_ENVIRONMENT_HPP

#include <qi/os.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

namespace Sinfonia
{
    namespace naoqi_env
    {

	static std::string getCMakePrefixPath()
	{
	    char *cMakePrefixPath = getenv( "CMAKE_PREFIX_PATH" );
	    if( cMakePrefixPath != NULL )
	    {
		return getenv( "CMAKE_PREFIX_PATH" );
	    }
	    return "";
	}

	static void adjustSDKPrefix()
	{
	    std::string cmakePrefix = getCMakePrefixPath();
	    std::vector<std::string> prefixes;
	    boost::split( prefixes, cmakePrefix, boost::is_any_of(":") );

	    for (size_t i=0; i<prefixes.size(); i++)
	    {
		std::cout << "going to add: " << prefixes[i] << std::endl;
		qi::path::detail::addOptionalSdkPrefix( prefixes[i].c_str() );
	    }
	}
    }
}
#endif
