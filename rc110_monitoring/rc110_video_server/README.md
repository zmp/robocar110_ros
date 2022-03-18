# Simple RTSP Video Server

A sample of streaming h265 over network.

Depends on **gstreamer-rtsp-server** ( **GPL3** ).
* https://github.com/GStreamer/gst-rtsp-server

![](docs/monitoring.jpg)

## Pipeline Scheme

![](docs/diagram.svg)

* [NVidia HW Encoder](https://developer.nvidia.com/nvidia-video-codec-sdk)

## Limitations

You need to stop camera node, before running this one.
```
nano ~/.config/rc110/service.conf
# Add one more parameter: 'roslaunch ... use_front_camera:=false'

systemctl --user restart rc110  # or from repo root: make start
```

* Maybe you'll need to restart robot to reconnect video device.

## Makefile
```
make deps            # check dependencies
make                 # build
make run             # run streaming on robot in VGA resolution
make run r=vga       # same (other values: hd, sxga, fhd, qhd)
make show            # playback the stream
```
`make show` has the following options:

* `host=zmp` - show camera on remote robot hosted as `zmp`
* `fps=on` - FPS overlay (only remote) (can crash with SIGSEGV)

URL to playback video stream with GUI application like VLC: `rtsp://192.168.110.5:8554/front`

## Parameters

```
debug_level (int, default: 1)
    debug level: 0 - no debug, 4 - max debug

port (string, default: "8554")
    RTSP port

url_suffix (string, default: front)
    device specific url suffix: rtsp://<>/front

video_device (string, default: video0)
    device file name: /dev/video0

max_framerate (int, default: 60)
    max FPS - can be lower due to low lighting
    
width (int, default: 1920)
    image width
    
height (int, default: 1080)
    image height
```

## Additional Info

### Test Results
For 1920x1080, 60fps:

* Throughput: < ~700 KB/s
* LAN Lag: < ~200 ms
* CPU Load: ~15% (1 core)
