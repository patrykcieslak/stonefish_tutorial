/*    
    This file is a part of stonefish_tutorial.

    stonefish_tutorial is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    stonefish_tutorial is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  ActuatorPlugin.cpp
//  stonefish_tutorial
//
//  Created by Patryk Cieslak on 10/11/2026.
//  Copyright (c) 2026 Patryk Cieslak. All rights reserved.
//

#include "stonefish_tutorial/ActuatorPlugin.h"

#include <entities/SolidEntity.h>

namespace sf
{

ActuatorPlugin::ActuatorPlugin(std::string_view uniqueName, Scalar amplitude, Scalar frequency)
    : LinkActuator(uniqueName), amplitude_{btFabs(amplitude)}, frequency_{btFabs(frequency)}, time_{0.0}
{
}

void ActuatorPlugin::Update(Scalar dt)
{
    Actuator::Update(dt);

    time_ += dt;
    Scalar force = amplitude_ * btSin(Scalar(2.0 * M_PI) * frequency_ * time_);
    
    if(attach_ != nullptr)
    {
        //Get transforms
        Transform solidTrans = attach_->GetCgTransform();
        Transform actuatorTrans = attach_->GetOTransform() * o2a_;
        
        attach_->ApplyCentralForce(actuatorTrans.getBasis() * Vector3(force, 0, 0));
        attach_->ApplyTorque((actuatorTrans.getOrigin() - solidTrans.getOrigin()).cross(actuatorTrans.getBasis() * Vector3(force, 0, 0)));
    }
}

void ActuatorPlugin::CreatePublishers(const std::shared_ptr<rclcpp::Node>& nh, 
    std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs, 
    std::unordered_map<std::string, std::string>& pubTopics) const
{
    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
        pubs[GetName()] = nh->create_publisher<std_msgs::msg::Float64>(it->second, 10);
}

void ActuatorPlugin::Publish(const std::shared_ptr<rclcpp::Node>& nh, 
    std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs) const
{
    (void)nh; // Suppress warning

    std_msgs::msg::Float64 msg;
    msg.data = frequency_;
    
    std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float64>>(pubs.at(GetName()))->publish(msg);
}

void ActuatorPlugin::CreateSubscribers(const std::shared_ptr<rclcpp::Node>& nh, 
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr>& subs, 
    std::unordered_map<std::string, std::string>& subTopics)
{
    if (auto it = subTopics.find("topic"); it != subTopics.end())
    {
        std::function<void(const std_msgs::msg::Float64::SharedPtr msg)> callbackFunc = 
            std::bind(&ActuatorPlugin::Callback, this, std::placeholders::_1);
        subs[GetName()] = nh->create_subscription<std_msgs::msg::Float64>(it->second, 10, callbackFunc);
    }
}

void ActuatorPlugin::Callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    amplitude_ = msg->data;
}

LinkActuatorType ActuatorPlugin::GetLinkActuatorType() const
{
    return LinkActuatorType::PLUGIN;
}

ConstructInfo ActuatorPlugin::GetConstructInfo()
{
    ConstructInfo info;
    ConstructInfoNode node;

    // Specs
    node.optional = false;
    node.attributes.insert({"amplitude", {ConstructInfoValueType::SCALAR, false}});
    node.attributes.insert({"frequency", {ConstructInfoValueType::SCALAR, false}});
    info.nodes.insert({"specs", node});

    return info;
}

ActuatorPlugin* ActuatorPlugin::Construct(std::string_view uniqueName, ConstructInfo* info)
{
    // Required
    Scalar amplitude = std::get<Scalar>(info->nodes.at("specs").attributes.at("amplitude").value);
    Scalar frequency = std::get<Scalar>(info->nodes.at("specs").attributes.at("frequency").value);

    return new ActuatorPlugin(uniqueName, amplitude, frequency);
}

extern "C" 
{
    Actuator* create(const char* uniqueName, ConstructInfo* info)
    {
        return ActuatorPlugin::Construct(uniqueName, info);
    }

    void destroy(Actuator* actuator) 
    {
        delete actuator;
    }

    ConstructInfo getConstructInfo()
    {
        return ActuatorPlugin::GetConstructInfo();
    }
}

}