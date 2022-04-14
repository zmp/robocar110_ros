/*
 * Copyright (C) 2022 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#pragma once

#include <memory>
#include <string>

namespace param_tools
{
/**
 * Similar to ros::Subscriber, but for parameters.
 */
class Subscriber
{
public:
    using Id = const std::string*;

public:
    Subscriber() {}
    Subscriber(std::string parameter) : parameter(new std::string(move(parameter))) {}
    ~Subscriber();

    Id getId() const { return parameter.get(); }
    const std::string& getParameter() const { return *parameter; }

private:
    std::shared_ptr<std::string> parameter;
};
}  // namespace param_tools