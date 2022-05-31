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
        parameters({
                .debugLevel = ros::param::param<int>("~debug_level", 1),
                .port = ros::param::param<int>("~port", 8554),
                .urlSuffix = ros::param::param<std::string>("~url_suffix", "front"),
                .videoDevice = ros::param::param<std::string>("~device", "/dev/video0"),
                .width = ros::param::param<int>("~width", 640),
                .height = ros::param::param<int>("~height", 480),
                .maxFrameRate = ros::param::param<int>("~fps", 30),
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
                       "%s --gst-debug=%d \"("
                       " nvv4l2camerasrc device=%s ! video/x-raw(memory:NVMM), framerate=%d/1, width=(int)%d, height=(int)%d"
                       //" v4l2src device=%s ! video/x-raw, framerate=%d/1, width=(int)%d, height=(int)%d"
                       // , if nvv4l2camerasrc won't work
                       " ! nvvidconv ! video/x-raw(memory:NVMM), format=(string)I420"
                       " ! omxh265enc ! video/x-h265, stream-format=(string)byte-stream"
                       " ! h265parse"
                       " ! rtph265pay name=pay0 pt=96 \")",
                       ros::this_node::getName().substr(1).c_str(),
                       parameters.debugLevel,
                       parameters.videoDevice.c_str(),
                       parameters.maxFrameRate,
                       parameters.width,
                       parameters.height);

    if (len < 0 || len >= MAX_LEN) {
        ROS_ERROR("Options string creation failed. Size: %d", len);
        return "";
    }

    ROS_INFO("Generated args:\n\t%s\n", optionsString);

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
        ROS_ERROR("Unable to parse options");
        g_clear_error(&error);
        return "";
    }

    if (!g_option_context_parse(options, &argc, &argv, &error)) {
        ROS_ERROR("Error parsing options: %s\n", error->message);
        g_option_context_free(options);
        g_clear_error(&error);
        return "";
    }
    g_option_context_free(options);

    if (argc < 2) {
        ROS_ERROR("No pipeline options");
        return "";
    }

    std::string pipelineOptions = argv[1];
    g_strfreev(argv);
    return pipelineOptions;
}

static void clientConnected(GstRTSPServer*, GstRTSPClient* client)
{
    GstRTSPConnection* connection = gst_rtsp_client_get_connection(client);
    ROS_INFO("Client connected: %s", gst_rtsp_connection_get_ip(connection));
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

    ROS_INFO("Stream ready at rtsp://<robot_ip>:%s/%s\n", portPointer, parameters.urlSuffix.c_str());
    return true;
}
}  // namespace zmp