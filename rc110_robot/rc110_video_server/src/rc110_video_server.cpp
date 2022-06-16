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
#include "rc110_video_server.hpp"

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/rtsp/gstrtspconnection.h>

namespace zmp
{
Rc110VideoServer::Rc110VideoServer() :
        Node("rc110_video_server"),
        parameters({
                (int)declare_parameter("debug_level", 1),
                (int)declare_parameter("port", 8554),
                declare_parameter("url_suffix", "front"),
                declare_parameter("gst_args", ""),
        }),
        portString(std::to_string(parameters.port)),
        portPointer(portString.data())
{
    if (!gstreamerInit()) {
        throw std::runtime_error("Unable to initialize gstreamer.");
    }
}

std::string Rc110VideoServer::parseOptions()
{
    static constexpr int MAX_LEN = 1000;
    char optionsString[MAX_LEN];
    int len = snprintf(optionsString,
                       MAX_LEN,
                       "%s --gst-debug=%d \"( %s \")",
                       &get_fully_qualified_name()[1],
                       parameters.debugLevel,
                       parameters.gstArgs.c_str());

    if (len < 0 || len >= MAX_LEN) {
        RCLCPP_ERROR(get_logger(), "Options string creation failed. Size: %d", len);
        return "";
    }

    RCLCPP_INFO(get_logger(), "Generated args: %s", optionsString);

    GOptionContext* options = g_option_context_new("");
    GOptionEntry entries[2] = {
            {"port", 'p', 0, G_OPTION_ARG_STRING, &portPointer, "Port to listen on", "PORT"},
            {nullptr},
    };

    g_option_context_add_main_entries(options, entries, nullptr);  // does memcpy of entries inside
    g_option_context_add_group(options, gst_init_get_option_group());

    int argc;
    char** argv;
    GError* error = nullptr;
    if (!g_shell_parse_argv(optionsString, &argc, &argv, &error)) {
        RCLCPP_ERROR(get_logger(), "Unable to parse options");
        g_clear_error(&error);
        return "";
    }

    if (!g_option_context_parse(options, &argc, &argv, &error)) {
        RCLCPP_ERROR(get_logger(), "Error parsing options: %s", error->message);
        g_option_context_free(options);
        g_clear_error(&error);
        return "";
    }
    g_option_context_free(options);

    if (argc < 2) {
        RCLCPP_ERROR(get_logger(), "No pipeline options");
        return "";
    }

    std::string pipelineOptions = argv[1];
    g_strfreev(argv);
    return pipelineOptions;
}

static void clientConnected(GstRTSPServer*, GstRTSPClient* client)
{
    GstRTSPConnection* connection = gst_rtsp_client_get_connection(client);
    RCLCPP_INFO(rclcpp::get_logger("gst"), "Client connected: %s", gst_rtsp_connection_get_ip(connection));
}

bool Rc110VideoServer::gstreamerInit()
{
    std::string pipelineOptions = parseOptions();  // Options starting with ( and finishing with )
    if (pipelineOptions.empty()) {
        return false;
    }

    g_main_loop_new(nullptr, false);

    GstRTSPServer* server = gst_rtsp_server_new();
    g_object_set(server, "service", portPointer, NULL);

    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server);

    GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(factory, pipelineOptions.c_str());
    gst_rtsp_media_factory_set_shared(factory, true);

    gst_rtsp_mount_points_add_factory(mounts, ("/" + parameters.urlSuffix).c_str(), factory);
    g_object_unref(mounts);

    gst_rtsp_server_attach(server, nullptr);
    g_signal_connect(server, "client-connected", (GCallback)clientConnected, nullptr);

    RCLCPP_INFO(get_logger(), "Stream ready at rtsp://<robot_ip>:%s/%s", portPointer, parameters.urlSuffix.c_str());
    return true;
}
}  // namespace zmp