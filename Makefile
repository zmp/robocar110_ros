.DEFAULT_GOAL := all
.ONESHELL:
SHELL := /bin/bash

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF

source:
ifeq (,$(shell grep "source /opt/ros" ~/.bashrc))
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
endif

all: source
	cd $$(catkin locate)
	catkin build ${cmake_flags}

package:
	cd $$(catkin locate)
	catkin build ${cmake_flags} -DCATKIN_BUILD_BINARY_PACKAGE=1

	function check_make_target {
		output=$$(make -n "$$1" 2>&1 | head -1)
		[[ "$$output" != *"No rule to make target"* ]]
	}

	cd $$(catkin locate --build)
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
	sudo apt install --reinstall ./*.deb

clean:
	cd $$(catkin locate)
	catkin clean -y

config:
	sudo ./rc110_launch/deb/postinst
