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
//  SensorPlugin.h
//  stonefish_tutorial
//
//  Created by Patryk Cieslak on 05/11/26.
//  Copyright (c) 2026 Patryk Cieslak. All rights reserved.

#pragma once

#include <Stonefish/sensors/scalar/LinkSensor.h>
#include <stonefish_ros2/ROS2SensorPluginInterface.h>

namespace sf
{
    //! A class representing an example of a sensor plugin that measures the surrounding fluid velocity.
    class SensorPlugin : public LinkSensor, public ROS2SensorPluginInterface
    {
    public:
        //! A constructor.
        /*!
        \param uniqueName a name for the sensor
        \param globalFrame a flag indicating whether the measurements are in the global frame
        \param frequency the sampling frequency of the sensor [Hz] (0 if updated every simulation step)
        \param historyLength defines: -1 -> no history, 0 -> unlimited history, >0 -> history with a specified length
        */
        SensorPlugin(std::string_view uniqueName, bool globalFrame, Scalar frequency = Scalar(-1), int historyLength = -1);

        void InternalUpdate(Scalar dt) override;

        //! A method used to set the noise characteristics of the sensor.
        /*!
        \param fluidVelocityStdDev standard deviation of the linear acceleration measurement noise for each axis
        */
        void SetNoise(Vector3 fluidVelocityStdDev);

        //! A method for creating ROS2 publishers for the sensor plugin.
        /*!
         \param nh a shared pointer to the ROS2 node handle
         \param pubs a map to store the created publishers, with sensor names as keys
         \param pubTopics a map storing all parsed publisher topics
         */
        void CreatePublishers(const std::shared_ptr<rclcpp::Node>& nh, 
            std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs, 
            std::unordered_map<std::string, std::string>& pubTopics) const override;

        //! A method for publishing sensor data.
        /*!
         \param nh a shared pointer to the ROS2 node handle
         \param pubs a map of the available publishers, with sensor names as keys
         */
        void Publish(const std::shared_ptr<rclcpp::Node>& nh, 
            std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs) const override;

        //! A method returning the type of scalar sensor.
        ScalarSensorType GetScalarSensorType() const override;

        //! A method returning the construction info for the sensor.
        static ConstructInfo GetConstructInfo();

        //! A method constructing the sensor based on info structure.
        /*!
        \param info a construction info structure
        */
        static SensorPlugin* Construct(std::string_view uniqueName, Scalar frequency, ConstructInfo* info);

    private:
        bool globalFrame_;
    };
}