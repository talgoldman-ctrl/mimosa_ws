/** -----------------------------------------------------------------------------
 * Copyright (c) 2023 Massachusetts Institute of Technology.
 * All Rights Reserved.
 *
 * AUTHORS:     Lukas Schmid <lschmid@mit.edu>, Nathan Hughes <na26933@mit.edu>
 * AFFILIATION: MIT-SPARK Lab, Massachusetts Institute of Technology
 * YEAR:        2023
 * LICENSE:     BSD 3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------------- */

#pragma once

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "config_utilities/factory.h"
#include "config_utilities/internal/string_utils.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/internal/yaml_utils.h"
#include "config_utilities/parsing/yaml.h"  // NOTE(lschmid): This pulls in more than needed buyt avoids code duplication.
#include "config_utilities/update.h"

namespace config {

namespace internal {

inline YAML::Node parameterToYaml(const rclcpp::Parameter& parameter) {
  switch (parameter.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return YAML::Node(parameter.as_bool());
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return YAML::Node(parameter.as_int());
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return YAML::Node(parameter.as_double());
    case rclcpp::ParameterType::PARAMETER_STRING: {
      const auto value = parameter.as_string();
      constexpr const char* yaml_tag = "__yaml__:";
      if (value.rfind(yaml_tag, 0) == 0) return YAML::Load(value.substr(9));
      return YAML::Node(value);
    }
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY: {
      YAML::Node node(YAML::NodeType::Sequence);
      for (const auto value : parameter.as_byte_array()) node.push_back(value);
      return node;
    }
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY: {
      YAML::Node node(YAML::NodeType::Sequence);
      for (const auto value : parameter.as_bool_array()) node.push_back(value);
      return node;
    }
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
      YAML::Node node(YAML::NodeType::Sequence);
      for (const auto value : parameter.as_integer_array()) node.push_back(value);
      return node;
    }
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
      YAML::Node node(YAML::NodeType::Sequence);
      for (const auto value : parameter.as_double_array()) node.push_back(value);
      return node;
    }
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY: {
      YAML::Node node(YAML::NodeType::Sequence);
      for (const auto& value : parameter.as_string_array()) node.push_back(value);
      return node;
    }
    default:
      return YAML::Node();
  }
}

// ROS 2 parameters are flat dotted names. Reconstruct the nested YAML tree
// expected by config_utilities.
inline YAML::Node rosToYaml(const rclcpp::Node& ros_node, const std::string& name_space = "") {
  YAML::Node yaml_node;
  const std::string prefix = name_space.empty() ? "" : name_space + ".";
  const auto result = ros_node.list_parameters(name_space.empty() ? std::vector<std::string>{} : std::vector<std::string>{name_space}, 1000);
  for (const auto& full_name : result.names) {
    if (!prefix.empty() && full_name.rfind(prefix, 0) != 0) continue;
    const std::string name = prefix.empty() ? full_name : full_name.substr(prefix.size());
    auto name_parts = splitNamespace(name, ".");
    if (name_parts.empty()) continue;
    const std::string local_name = name_parts.back();
    name_parts.pop_back();
    YAML::Node local_node;
    local_node[local_name] = parameterToYaml(ros_node.get_parameter(full_name));
    moveDownNamespace(local_node, joinNamespace(name_parts));
    mergeYamlNodes(yaml_node, local_node);
  }
  return yaml_node;
}

}  // namespace internal

/**
 * @brief Loads a config from a yaml node.
 *
 * @tparam ConfigT The config type. This can also be a VirtualConfig<BaseT> or a std::vector<ConfigT>.
 * @param nh The ROS nodehandle to create the config from.
 * @param name_space Optionally specify a name space to create the config from. Separate names with slashes '/'.
 * Example: "my_config/my_sub_config".
 * @returns The config.
 */
template <typename ConfigT>
ConfigT fromRos(const rclcpp::Node& ros_node, const std::string& name_space = "") {
  const YAML::Node node = internal::rosToYaml(ros_node, name_space);
  return fromYaml<ConfigT>(node, "");
}

/**
 * @brief Create a derived type object based on a the data stored in a yaml node. All derived types need to be
 * registered to the factory using a static config::Registration<BaseT, DerivedT, ConstructorArguments...> struct. They
 * need to implement a config as a public member struct named 'Config' and use the config as the first constructor
 * argument.
 *
 * @tparam BaseT Type of the base class to be constructed.
 * @tparam Args Other constructor arguments. Note that each unique set of constructor arguments will result in a
 * different base-entry in the factory.
 * @param nh The ROS nodehandle containing the type identifier as a param and the data to create the config.
 * @param args Other constructor arguments.
 * @returns Unique pointer of type base that contains the derived object.
 */
template <typename BaseT, typename... ConstructorArguments>
std::unique_ptr<BaseT> createFromROS(const rclcpp::Node& node, ConstructorArguments... args) {
  return internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::create(internal::rosToYaml(node), args...);
}

/**
 * @brief Create a derived type object based on a the data stored in a yaml node. All derived types need to be
 * registered to the factory using a static config::Registration<BaseT, DerivedT, ConstructorArguments...> struct. They
 * need to implement a config as a public member struct named 'Config' and use the config as the first constructor
 * argument.
 *
 * @tparam BaseT Type of the base class to be constructed.
 * @tparam Args Other constructor arguments. Note that each unique set of constructor arguments will result in a
 * different base-entry in the factory.
 * @param nh The ROS nodehandle containing the type identifier as a param and the data to create the config.
 * @param name_space Optionally specify a name space to create the object from. Separate names with
 * slashes '/'. Example: "my_config/my_sub_config".
 * @param args Other constructor arguments.
 * @returns Unique pointer of type base that contains the derived object.
 */
template <typename BaseT, typename... ConstructorArguments>
std::unique_ptr<BaseT> createFromROSWithNamespace(const rclcpp::Node& node,
                                                  const std::string& name_space,
                                                  ConstructorArguments... args) {
  return internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::create(
      internal::rosToYaml(node, name_space), args...);
}

/**
 * @brief Update the config with the current parameters in ROS.
 * @note This function will update the field and check the validity of the config afterwards. If the config is invalid,
 * the field will be reset to its original value.
  * @param config The config to update.
  * @param nh The ROS nodehandle to update the config from.
  * @param name_space Optionally specify a name space to create the config from. Separate names with slashes '/'.
 */
template <typename ConfigT>
bool updateFromRos(ConfigT& config, const rclcpp::Node& ros_node, const std::string& name_space = "") {
  const YAML::Node node = internal::rosToYaml(ros_node, name_space);
  return updateField(config, node, true, name_space);
}

}  // namespace config
