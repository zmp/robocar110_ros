/*
 * Copyright (C) 2022 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#pragma once

#include <xmlrpcpp/XmlRpc.h>

#include <functional>
#include <map>
#include <string>

#include "subscriber.hpp"

namespace param_tools
{
/**
 * A class providing subscription to ros parameter updates.
 *
 * @example
 *   subscriber = param_tools::instance().subscribe("~param_name", [this](const XmlRpc::XmlRpcValue& value) { ... });
 */
class ParamTools
{
    using CallbackType = std::function<void(const XmlRpc::XmlRpcValue&)>;
    friend ParamTools& instance();

    ParamTools();

public:
    /**
     * Subscribe to parameter change. It can be called multiple times on the same parameter.
     * @return Subscription instance. Unsubscription happens when its last copy is destructed.
     */
    Subscriber subscribe(const std::string& parameter, CallbackType callback);

    /**
     * Unsubscribe given subscriber (called automatically on the last subscriber copy destruction).
     */
    void unsubscribe(Subscriber* subscriber);

    /**
     * Emulate topic publish behavior for parameter, so not only other nodes get callback, but the current node too.
     */
    void publish(const std::string& parameter, const XmlRpc::XmlRpcValue& value);

    ParamTools(const ParamTools&) = delete;
    void operator=(const ParamTools&) = delete;

private:
    void sendAll(const std::string& parameter, const XmlRpc::XmlRpcValue& value);

private:
    std::map<std::string, std::map<Subscriber::Id, CallbackType>> parameterCallbacks;
};

/** Singletone */
ParamTools& instance();
}  // namespace param_tools