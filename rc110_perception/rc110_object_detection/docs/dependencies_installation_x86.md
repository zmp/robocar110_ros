## Dependencies Installation on x86 PC

JetPack provides the dependencies out of box, but if you want to build on remote PC as well, additional steps are needed **on the remote PC**.

### Prepare nvidia repositories
```
uver=1804  # 2004 for Ubuntu 20.04

sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu${uver}/x86_64/7fa2af80.pub

sudo tee -a /etc/apt/sources.list.d/nvidia.list > /dev/null <<EOT
deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu${uver}/x86_64 /
deb http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu${uver}/x86_64 /
EOT

sudo apt update
```

### Install the dependencies
```
cver=10-2              # around 11-4             for Ubuntu 20
nver=8.0.1-1+cuda10.2  # around 8.2.5-1+cuda11.4 for Ubuntu 20

sudo apt install cuda-${cver} libnvinfer-dev=${nver} libnvinfer-plugin-dev=${nver} libnvparsers-dev=${nver} libnvonnxparsers-dev=${nver} libnvparsers8=${nver} libnvinfer8=${nver} libnvinfer-plugin8=${nver} libnvonnxparsers8=${nver}
```

* Change `10-2` to the version shipped with your JetPack.
* Here `8.0.1-1` can be changed to the latest version compatible with your cuda version.
* Probably `nvidia-driver` needs to be upgraded in order to install `cuda`.

### Notes
* Make sure to set the correct version, because your build will fail otherwise.
* If you had other CUDA versions, either remove them or configure the correct link: `/usr/local/cuda -> /usr/local/cuda-10.2`
