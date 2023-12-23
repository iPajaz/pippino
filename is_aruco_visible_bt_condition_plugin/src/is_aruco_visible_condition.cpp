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

#include <string>

#include "is_aruco_visible_bt_condition_plugin/is_aruco_visible_condition.hpp"

namespace nav2_behavior_tree
{

    IsArucoVisibleCondition::IsArucoVisibleCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          aruco_visible_topic_("/aruco_visible"),
          is_aruco_visible_(false)
    {
        getInput("aruco_visible_topic", aruco_visible_topic_);
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        aruco_visible_sub_ = node_->create_subscription<aruco_interface::msg::VisibleArucoMarkers>(
            aruco_visible_topic_,
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IsArucoVisibleCondition::arucoVisibleCallback, this, std::placeholders::_1),
            sub_option);
    }

    BT::NodeStatus IsArucoVisibleCondition::tick()
    {
        callback_group_executor_.spin_some();
        if (is_aruco_visible_)
        {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    void IsArucoVisibleCondition::arucoVisibleCallback(aruco_interface::msg::VisibleArucoMarkers::SharedPtr msg)
    {
        // RCLCPP_INFO(this->node_->get_logger(), "Helloo!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        // RCLCPP_INFO(this->node_->get_logger(), "I heard: '%d'", msg->visible_markers.size());
        is_aruco_visible_ = msg->visible_markers.size() > 0;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IsArucoVisibleCondition>("IsArucoVisible");
}
