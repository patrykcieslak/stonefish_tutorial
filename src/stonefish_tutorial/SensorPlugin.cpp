/*    
    This file is a part of stonefish_tutorial.

    stonefish_ros is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    stonefish_ros is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  SensorPlugin.cpp
//  stonefish_tutorial
//
//  Created by Patryk Cieslak on 05/11/26.
//  Copyright (c) 2026 Patryk Cieslak. All rights reserved.

#include "stonefish_tutorial/SensorPlugin.h"

#include <Stonefish/core/SimulationApp.h>
#include <Stonefish/core/SimulationManager.h>
#include "geometry_msgs/msg/velocity_stamped.hpp"

namespace sf
{

SensorPlugin::SensorPlugin(std::string_view uniqueName, bool globalFrame, Scalar frequency, int historyLength)
        : LinkSensor(uniqueName, frequency, historyLength), ROS2SensorPluginInterface(), globalFrame_(globalFrame)
{
    channels_.push_back(SensorChannel("Fluid velocity X", QuantityType::VELOCITY));
    channels_.push_back(SensorChannel("Fluid velocity Y", QuantityType::VELOCITY));
    channels_.push_back(SensorChannel("Fluid velocity Z", QuantityType::VELOCITY));
}

ScalarSensorType SensorPlugin::GetScalarSensorType() const
{ 
    return ScalarSensorType::PLUGIN; 
}

void SensorPlugin::SetNoise(Vector3 fluidVelocityStdDev)
{
    channels_[0].SetStdDev(btClamped(fluidVelocityStdDev.getX(), Scalar(0), Scalar(BT_LARGE_FLOAT)));
    channels_[1].SetStdDev(btClamped(fluidVelocityStdDev.getY(), Scalar(0), Scalar(BT_LARGE_FLOAT)));
    channels_[2].SetStdDev(btClamped(fluidVelocityStdDev.getZ(), Scalar(0), Scalar(BT_LARGE_FLOAT)));
}

void SensorPlugin::InternalUpdate(Scalar dt)
{
    Vector3 data(0.0, 0.0, 0.0); //Fluid velocity [m/s]
    
    Ocean* liq = SimulationApp::GetApp()->GetSimulationManager()->GetOcean();
    if(liq != nullptr && liq->IsInsideFluid(GetSensorFrame().getOrigin()))
    {
        if(globalFrame_)
            data = liq->GetFluidVelocity(GetSensorFrame().getOrigin());
        else
            data = GetSensorFrame().getBasis().inverse() * liq->GetFluidVelocity(GetSensorFrame().getOrigin());
    }    
    
    //Record sample
    AddSampleToHistory(std::make_unique<Sample>(std::vector<Scalar>({data.getX(), data.getY(), data.getZ()})));
}

void SensorPlugin::CreatePublishers(const std::shared_ptr<rclcpp::Node>& nh, 
            std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs, 
            std::unordered_map<std::string, std::string>& pubTopics) const
{
    unsigned int queueSize = (unsigned int)ceil(GetUpdateFrequency());
    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
        pubs[GetName()] = nh->create_publisher<geometry_msgs::msg::VelocityStamped>(it->second, queueSize);
}

void SensorPlugin::Publish(const std::shared_ptr<rclcpp::Node>& nh, 
    std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs) const
{
    Sample s = GetLastSample();

    geometry_msgs::msg::VelocityStamped msg;
    msg.header.stamp = nh->get_clock()->now();
    msg.header.frame_id = globalFrame_ ? "world_ned" : GetName();
    msg.body_frame_id = GetName();
    msg.reference_frame_id = msg.header.frame_id;
    msg.velocity.linear.x = s.GetValue(0);
    msg.velocity.linear.y = s.GetValue(1);
    msg.velocity.linear.z = s.GetValue(2);
    
    std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::VelocityStamped>>(pubs.at(GetName()))->publish(msg);
}

// Statics
ConstructInfo SensorPlugin::GetConstructInfo()
{
    ConstructInfo info;
    ConstructInfoNode node;
    
    // History
    node.optional = true;
    node.attributes.insert({"samples", {ConstructInfoValueType::INT, false}});
    info.nodes.insert({"history", node});

    // Specs
    node.attributes.clear();
    node.optional = false;
    node.attributes.insert({"global_frame", {ConstructInfoValueType::BOOL, false}});
    info.nodes.insert({"specs", node});

    // Noise
    node.attributes.clear(); // Clear temporary
    node.optional = true;
    node.attributes.insert({"fluid_velocity", {ConstructInfoValueType::VECTOR3, false}});
    info.nodes.insert({"noise", node});
    
    return info;
}

SensorPlugin* SensorPlugin::Construct(std::string_view uniqueName, Scalar frequency, ConstructInfo* info)
{
    // History (optional)
    int history = -1;
    ConstructInfoValue& value = info->nodes.at("history").attributes.at("samples");
    if (value.valid)
        history = std::get<int>(value.value);

    // Specs
    bool globalFrame = std::get<bool>(info->nodes.at("specs").attributes.at("global_frame").value);
    
    // Create sensor
    SensorPlugin* sensor = new SensorPlugin(uniqueName, globalFrame, frequency, history);    

    // Noise (optional)
    value = info->nodes.at("noise").attributes.at("fluid_velocity");
    if (value.valid)
        sensor->SetNoise(std::get<Vector3>(value.value));

    return sensor;
}

extern "C" 
{
    Sensor* create(const char* uniqueName, Scalar frequency, ConstructInfo* info)
    {
        return SensorPlugin::Construct(uniqueName, frequency, info);
    }

    void destroy(Sensor* sensor) 
    {
        delete sensor;
    }

    ConstructInfo getConstructInfo()
    {
        return SensorPlugin::GetConstructInfo();
    }
}

}