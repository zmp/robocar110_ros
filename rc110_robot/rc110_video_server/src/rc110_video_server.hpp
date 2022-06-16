/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Written by Andrei Pak
 */
#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace zmp
{
class Rc110VideoServer : public rclcpp::Node
{
    struct Parameters {
        int debugLevel;
        int port;
        std::string urlSuffix;
        std::string gstArgs;
    };

public:
    Rc110VideoServer();

private:
    std::string parseOptions();
    bool gstreamerInit();

private:
    Parameters parameters;
    std::string portString;
    char* portPointer;  // valid pointer is needed for GOptionEntry
};
}  // namespace zmp