/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Paul Mathieu. */

#ifndef TWIST_MUX_XMLRPC_HELPERS_H
#define TWIST_MUX_XMLRPC_HELPERS_H

#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <map>
#include <variant>

namespace xh
{

class XmlrpcHelperException : public std::runtime_error
{
public:
  explicit XmlrpcHelperException(const std::string& what)
    : std::runtime_error(what) {}
};

// ROS 2 parameter types - simplified version of XmlRpc functionality
using ParameterValue = std::variant<bool, int64_t, double, std::string, 
                                   std::vector<uint8_t>, std::vector<bool>, 
                                   std::vector<int64_t>, std::vector<double>, 
                                   std::vector<std::string>>;

using Struct = std::map<std::string, ParameterValue>;
using Array = std::vector<ParameterValue>;

template <class T>
void fetchParam(rclcpp::Node::SharedPtr node, const std::string& param_name, T& output)
{
  try 
  {
    // Declare parameter if it doesn't exist
    if (!node->has_parameter(param_name)) 
    {
      node->declare_parameter(param_name, rclcpp::ParameterValue{});
    }

    rclcpp::Parameter param = node->get_parameter(param_name);
    
    if (param.get_type() == rclcpp::PARAMETER_NOT_SET) 
    {
      std::ostringstream err_msg;
      err_msg << "could not load parameter '" << param_name << "'. (node: "
        << node->get_name() << ")";
      throw XmlrpcHelperException(err_msg.str());
    }

    // Convert parameter to desired type
    convertParameter(param, output);
  }
  catch (const rclcpp::exceptions::ParameterNotDeclaredException& e)
  {
    std::ostringstream err_msg;
    err_msg << "parameter '" << param_name << "' not declared. (node: "
      << node->get_name() << ")";
    throw XmlrpcHelperException(err_msg.str());
  }
}

// Helper function to convert ROS 2 parameters to desired types
template<typename T>
void convertParameter(const rclcpp::Parameter& param, T& output)
{
  try 
  {
    if constexpr (std::is_same_v<T, bool>) 
    {
      output = param.as_bool();
    }
    else if constexpr (std::is_same_v<T, int> || std::is_same_v<T, int64_t>) 
    {
      output = param.as_int();
    }
    else if constexpr (std::is_same_v<T, double>) 
    {
      output = param.as_double();
    }
    else if constexpr (std::is_same_v<T, std::string>) 
    {
      output = param.as_string();
    }
    else if constexpr (std::is_same_v<T, std::vector<bool>>) 
    {
      output = param.as_bool_array();
    }
    else if constexpr (std::is_same_v<T, std::vector<int64_t>>) 
    {
      output = param.as_integer_array();
    }
    else if constexpr (std::is_same_v<T, std::vector<double>>) 
    {
      output = param.as_double_array();
    }
    else if constexpr (std::is_same_v<T, std::vector<std::string>>) 
    {
      output = param.as_string_array();
    }
    else 
    {
      throw XmlrpcHelperException("Unsupported parameter type conversion");
    }
  }
  catch (const rclcpp::ParameterTypeException& e) 
  {
    std::ostringstream err_msg;
    err_msg << "parameter type conversion failed: " << e.what();
    throw XmlrpcHelperException(err_msg.str());
  }
}

void checkArrayItem(const Array& col, int index)
{
  if (index < 0 || index >= static_cast<int>(col.size()))
  {
    std::ostringstream err_msg;
    err_msg << "index '" << index << "' is out of array bounds [0, " << col.size() << ")";
    throw XmlrpcHelperException(err_msg.str());
  }
}

void checkStructMember(const Struct& col, const std::string& member)
{
  if (col.find(member) == col.end())
  {
    std::ostringstream err_msg;
    err_msg << "could not find member '" << member << "'";
    throw XmlrpcHelperException(err_msg.str());
  }
}

template <class T>
void getArrayItem(const Array& col, int index, T& output)
{
  checkArrayItem(col, index);
  
  try 
  {
    if (auto val = std::get_if<T>(&col[index])) 
    {
      output = *val;
    } 
    else 
    {
      throw XmlrpcHelperException("Array item type mismatch");
    }
  }
  catch (const std::bad_variant_access& e) 
  {
    throw XmlrpcHelperException("Array item type access error");
  }
}

template <class T>
void getStructMember(const Struct& col, const std::string& member, T& output)
{
  checkStructMember(col, member);
  
  try 
  {
    if (auto val = std::get_if<T>(&col.at(member))) 
    {
      output = *val;
    } 
    else 
    {
      throw XmlrpcHelperException("Struct member type mismatch");
    }
  }
  catch (const std::bad_variant_access& e) 
  {
    throw XmlrpcHelperException("Struct member type access error");
  }
}

// Alternative simpler approach for basic parameter fetching
template <class T>
void fetchSimpleParam(rclcpp::Node::SharedPtr node, const std::string& param_name, T& output)
{
  node->declare_parameter(param_name, T{});
  node->get_parameter(param_name, output);
}

} // namespace xh

#endif // TWIST_MUX_XMLRPC_HELPERS_H