#
# Convenient script to use with make on Linux.
# To check the commands list, input: make<space><tab>
#
include mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF
main_nodes := rc110_system rc110_rviz

define help_text

GNUmakefile provides the following targets for make:

    version                   Show current versions of source and packages
    ros-install               ROS installation
    ros-source                Add ros environment to .bashrc

    deps                      Install dependencies for rc110_core
    deps-%                    Install dependencies for %
    all (empty)               Build rc110_core
    %                         Build %
    package                   Create deb packages in build/ directory
    install                   Install the packages to system
    self                      Create "run" installation file
    clean                     Clean build and package files

    start                     Restart RoboCar nodes from system
    stop                      Stop RoboCar nodes from system
    run                       Run rc110_core from build (Don't forget to stop system nodes first!)
    run-%                     Run % node from build
    show                      Show general RViz on robot
    show-%                    Show % RViz on robot

    env                       Create env.sh file for connection to remote PC (Needs to be edited!)
    monitor                   Show general RViz on remote PC
    monitor-%                 Show % RViz on remote PC
    remote-joy                Run joystick connected to remote PC (By default it's connected to RoboCar)

    save-map                  Save Hector SLAM map   [map_name=map]
    select-map                Select Hector SLAM map [map_name=map]
    camera-calibration-file   Setup calibration from archive  (For details, see: CameraCalibration.md)

endef

# == targets ==

export help_text
help:
	@echo -e "\033[1;36m$${help_text}\033[0m"

# Show versions
version:
	@echo -e "\
	Tegra: $$(cat /etc/nv_tegra_release 2> /dev/null) \n\
	Last JetPack: $$(apt-cache show nvidia-jetpack 2> /dev/null | grep Version | awk 'NR==1{print $$2}') \n\
	Base Driver:  $$(dpkg -s rc-system 2> /dev/null | grep Version | awk '{print $$2}') \n\
	Core Nodes:   $$(dpkg -s ros-${ROS_DISTRO}-rc110-msgs 2> /dev/null | grep Version | awk '{print $$2}') \n\
	Source Code:  $$(grep -oPm1 '(?<=<version>)[^<]+' rc110_core/rc110_common/package.xml) "

# Shortcut for ROS installation.
ros-install:
	./scripts/install_ros

# Add ros environment to .bashrc.
ros-source:
	@
ifeq (,$(shell grep -q "source /opt/ros" ~/.bashrc))
	echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
	echo "ROS sourcing is added. Please, restart the shell to apply."
else
	echo "ROS sourcing exists already."
endif

# Install rosdep and get packages list.
init-deps:
ifeq (,$(shell which rosdep))
	sudo apt-get install -qq python-rosdep
endif
ifeq (,$(wildcard /etc/ros/rosdep/sources.list.d/20-default.list))
	sudo rosdep init
	rosdep update --rosdistro=${ROS_DISTRO}
endif

# Install core dependencies.
deps: init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths rc110_core --skip-keys=rc110_master_hold

# Init catkin workspace.
init:
ifeq (,$(shell which catkin))
	sudo apt-get install -qq python-catkin-tools
endif
ifeq (src,$(notdir $(abspath ..)))
  ifeq (,$(wildcard ../../.catkin_tools))
	cd ../..
	catkin config --init --log-space .catkin_tools/logs
  endif
else
  ifeq (,$(wildcard .catkin_tools))
	@echo -e "\033[1;31m\
	    \nWarning: Catkin workspace will be created in the current directory!\
	    \nCreate and use rc110_contrib/ directory for new nodes.\
	    \nIf you want to have usual workspace, please, put robocar110_ros to src/\
	    \n\033[0m"
	catkin config --init --source-space . --log-space .catkin_tools/logs
  endif
endif

# Build all.
all: init
	@source /opt/ros/${ROS_DISTRO}/setup.bash
	$(call build,${main_nodes},${cmake_flags})

# Make packages in build directory.
package: init
	@source /opt/ros/${ROS_DISTRO}/setup.bash
	$(call build,${main_nodes},${cmake_flags} -DCATKIN_BUILD_BINARY_PACKAGE=1)

	function check_make_target {
		output=$$(make -n "$$1" -C "$$2" 2>&1 || true | head -1)
		[[ "$$output" != *"No rule to make target"* ]]
	}

	cd $$(catkin locate --build)
	rm -f *.deb
	for d in */
	do
		(  # parallel run
		if check_make_target package "$$d"
		then
			cd "$$d"
				$(MAKE) package >/dev/null
				mv *.deb ../
				echo "Package created: $$d"
			cd ..
		fi
		) &
	done
	wait

# Install packages to system folder.
install: package
	source /opt/ros/${ROS_DISTRO}/setup.bash
	cd $$(catkin locate --build)
	sudo dpkg --configure -a  # resolves "Internal Error, No file name for"
	sudo apt-get install -qq --allow-downgrades --reinstall ./*.deb
	systemctl --user daemon-reload  # automatic files reload - it does not work from postinst, as root runs postinst

# Self-extracting archive with core packages.
self: package
ifeq (,$(shell which makeself))
	sudo apt-get install -qq makeself
endif

	root_dir=$$(pwd)
	source /opt/ros/${ROS_DISTRO}/setup.bash
	cd $$(catkin locate --build)
	
	version=$$(dpkg-deb -f $$(ls *rc110*.deb | head -1) Version)
	rm -rf stage; mkdir stage
	mv *.deb stage/
	cp $${root_dir}/scripts/install* stage/

	makeself stage rc110_core_$${version}.run "package" ./install

# Clean all.
clean:
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin clean -y

# Environment variables for remote access.
env:
ifeq (,$(wildcard ../../env.sh))
	cp mk/env_template.sh ../../env.sh
endif

# Prepare variables for run-* targets.
init-run:
ifeq (,$(wildcard ~/.config/rc110/service.conf))
	cp rc110_core/rc110_system/deb/service_template.conf ~/.config/rc110/service.conf
endif

# Run nodes built from source.
run: init-run
	systemctl --user start rc110-roscore
	source ../../devel/setup.bash
	source ~/.config/rc110/service.conf
	eval "$$RC110_LAUNCH_COMMAND"

# Run only joystick node on remote PC.
remote-joy: env
	source ../../env.sh
	$(MAKE) run -C rc110_core/rc110_teleop


# == additional targets ==

include mk/subdirs.mk

# == shortcuts ==

show: show-rviz
monitor: monitor-rviz
