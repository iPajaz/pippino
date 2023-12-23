// Copyright (c) 2023 Michele De Marchi
// Copyright (c) 2020 Sarthak Mittal
// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ARUCO_VISIBLE_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ARUCO_VISIBLE_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "aruco_interface/msg/visible_aruco_markers.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a aruco visible topic and
     * returns SUCCESS when aruco is visible and FAILURE otherwise
     */
    class IsArucoVisibleCondition : public BT::ConditionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsArucoVisibleCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        IsArucoVisibleCondition(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        IsArucoVisibleCondition() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        /**
         * @brief Creates list of BT ports
         * @return BT::PortsList Containing node-specific ports
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>(
                    "aruco_visible_topic", std::string("/aruco_visible"), "Aruco visible topic"),
            };
        }

    private:
        /**
         * @brief Callback function for aruco visible topic
         * @param msg Shared pointer to aruco_interface:msg:VisibleArucoMarkers message
         */
        void arucoVisibleCallback(aruco_interface::msg::VisibleArucoMarkers::SharedPtr msg);

        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<aruco_interface::msg::VisibleArucoMarkers>::SharedPtr aruco_visible_sub_;
        std::string aruco_visible_topic_;
        bool is_aruco_visible_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ARUCO_VISIBLE_CONDITION_HPP_
