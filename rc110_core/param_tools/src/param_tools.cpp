/*
 * Copyright (C) 2022 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include "../include/param_tools/param_tools.hpp"

#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>

namespace ros::param
{
void paramUpdateCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
}

namespace param_tools
{
ParamTools::ParamTools()
{
    ros::XMLRPCManager::instance()->unbind("paramUpdate");
    ros::XMLRPCManager::instance()->bind("paramUpdate", [this](XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
        ros::param::paramUpdateCallback(params, result);

        sendAll(params[1], params[2]);
    });
}

Subscriber ParamTools::subscribe(const std::string& parameter, std::function<void(const XmlRpc::XmlRpcValue&)> callback)
{
    if (parameter.empty()) {
        return {};
    }
    // Callback should be called also, if subscription happens after parameter was set.
    if (XmlRpc::XmlRpcValue value; ros::param::get(parameter, value)) {
        callback(value);
    }

    auto globalParameter = ros::names::resolve(parameter);
    XmlRpc::XmlRpcValue request, response, payload;
    request[0] = ros::this_node::getName();
    request[1] = ros::XMLRPCManager::instance()->getServerURI();
    request[2] = globalParameter;
    if (ros::master::execute("subscribeParam", request, response, payload, false)) {
        Subscriber subscriber(globalParameter);
        parameterCallbacks[globalParameter].emplace(subscriber.getId(), move(callback));
        return subscriber;
    }
    return {};
}

void ParamTools::unsubscribe(Subscriber* subscriber)
{
    auto callbackGroup = parameterCallbacks[subscriber->getParameter()];
    callbackGroup.erase(subscriber->getId());
    if (callbackGroup.empty()) {
        parameterCallbacks.erase(subscriber->getParameter());
    }
}

void ParamTools::publish(const std::string& parameter, const XmlRpc::XmlRpcValue& value)
{
    ros::param::set(parameter, value);
    sendAll(parameter, value);
}

void ParamTools::sendAll(const std::string& parameter, const XmlRpc::XmlRpcValue& value)
{
    auto globalParameter = ros::names::resolve(parameter);
    if (parameterCallbacks.count(globalParameter)) {
        for (auto& idCallback : parameterCallbacks[globalParameter]) {
            idCallback.second(value);
        }
    }
}

ParamTools& instance()
{
    static ParamTools instance;
    return instance;
}
}  // namespace param_tools