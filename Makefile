.DEFAULT_GOAL := all
.ONESHELL:
.SHELLFLAGS := -ec
SHELL := /bin/bash

# catkin sometimes fail to find number of cores
build_cores := $(shell ps T | sed -n 's/.*$(MAKE_PID).*$(MAKE).* \(-j\|--jobs\) *\([0-9][0-9]*\).*/\2/p')
ifeq ($(build_cores),)
	build_cores := $(shell grep -c ^processor /proc/cpuinfo)
endif
cmake_flags := -j$(build_cores) -DCATKIN_ENABLE_TESTING=OFF

source:
ifeq (,$(shell grep "source /opt/ros" ~/.bashrc))
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
endif

all: source
	catkin build ${cmake_flags}

package:
	catkin build ${cmake_flags} -DCATKIN_BUILD_BINARY_PACKAGE=1

	function check_make_target {
		output=$$(make -n "$$1" 2>&1 | head -1)
		[[ "$$output" != *"No rule to make target"* ]]
	}

	cd $$(catkin locate --build)
	rm -f *.deb
	for d in */
	do
		(  # make package in parallel
		pushd "$$d"
			if check_make_target package; then
				$(MAKE) package
				mv *.deb ../
			fi
		popd
		) &
	done
	wait

install: package
	cd $$(catkin locate --build)
	sudo apt-get install --reinstall ./*.deb
	systemctl --user daemon-reload  # automatic files reload - it does not work from postinst, as root runs postinst

clean:
	catkin clean -y

config:
	sudo ./rc110_launch/deb/postinst
