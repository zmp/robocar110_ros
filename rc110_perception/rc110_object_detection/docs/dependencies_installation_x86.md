## Dependencies Installation on x86 PC

JetPack provides the dependencies out of box, but if you want to build on remote PC as well, additional steps are needed **on the remote PC**.

* The following steps are for **Ubuntu 18**.
* Prepare nvidia repositories:
```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-repo-ubuntu1804_10.1.243-1_amd64.deb
sudo apt install ./cuda-repo-ubuntu1804_10.1.243-1_amd64.deb
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub

wget http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/nvidia-machine-learning-repo-ubuntu1804_1.0.0-1_amd64.deb
sudo apt install ./nvidia-machine-learning-repo-ubuntu1804_1.0.0-1_amd64.deb
sudo apt-get update
```

* Install the dependencies:
```
sudo apt-get install cuda-10-2 libnvinfer-dev=7.2.3-1+cuda10.2 libnvinfer-plugin-dev=7.2.3-1+cuda10.2 libnvparsers-dev=7.2.3-1+cuda10.2 libnvonnxparsers-dev=7.2.3-1+cuda10.2
```
* Change 10-2 to the version shipped with your JetPack.
* Here 7.2.3-1 can be changed to the latest version compatible with your cuda version.

#### Notes
* Make sure to set the correct version, because your build will fail otherwise.
* If you had other CUDA versions, either remove them or configure the correct link: `/usr/local/cuda -> /usr/local/cuda-10.2`
