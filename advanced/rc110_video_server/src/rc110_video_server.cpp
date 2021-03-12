/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 * All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
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
        handlePrivate("~"),
        parameters({
                .debugLevel = handlePrivate.param<int>("debug_level", 1),
                .port = handlePrivate.param<std::string>("port", "8554"),
                .urlSuffix = handlePrivate.param<std::string>("url_suffix", "front"),
                .videoDevice = handlePrivate.param<std::string>("video_device", "video0"),
                .maxFrameRate = handlePrivate.param<int>("max_framerate", 60),
                .width = handlePrivate.param<int>("width", 1920),
                .height = handlePrivate.param<int>("height", 1080),
        }),
        portPointer(parameters.port.data())
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
                       " nvv4l2camerasrc device=/dev/%s ! video/x-raw(memory:NVMM), framerate=%d/1, width=(int)%d, height=(int)%d"
                       //" v4l2src device=/dev/%s ! video/x-raw, framerate=%d/1, width=(int)%d, height=(int)%d"  // if nvv4l2camerasrc won't work
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
    entries.reset(new GOptionEntry[2]{
            {"port", 'p', 0, G_OPTION_ARG_STRING, &portPointer, "Port to listen on", "PORT"},
            {nullptr},
    });

    g_option_context_add_main_entries(options, entries.get(), nullptr);
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