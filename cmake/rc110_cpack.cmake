if(NOT DEFINED ENV{ROS_DISTRO})
    message(FATAL_ERROR "ROS_DISTRO is not defined. Please, source ros.")
endif()

#
# project name to debian package name
#
function(to_deb_name result name)
    string(REGEX REPLACE "_" "-" name ${name})
    string(TOLOWER "${name}" name)
    set(${result} ${name} PARENT_SCOPE)
endfunction()

#
# Convert ros dependency name to debian package name.
#
function(ros_to_deb_name result dep)
    # Some packages can be found with rosdep resolve.
    execute_process(
            COMMAND rosdep resolve ${dep}
            COMMAND sed -n 2p  # 2nd line
            OUTPUT_VARIABLE dep0
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULTS_VARIABLE results
    )
    list(GET results 0 rosdep_result)
    if(rosdep_result EQUAL 0)
        set(${result} ${dep0} PARENT_SCOPE)
        return()
    endif()

    # Rosdep cannot look for non listed packages in resolve mode, so we check it manually.
    to_deb_name(dep1 ros-$ENV{ROS_DISTRO}-${dep})
    execute_process(COMMAND apt-cache search --names-only ^${dep1}$ OUTPUT_VARIABLE output)
    if(NOT output STREQUAL "")
        set(${result} ${dep1} PARENT_SCOPE)
        return()
    endif()

    # Some use debian package name as is. For example usual packages, that are not in rosdep yaml list.
    to_deb_name(dep2 ${dep})
    execute_process(COMMAND apt-cache search --names-only ^${dep2}$ OUTPUT_VARIABLE output)
    if(NOT output STREQUAL "")
        set(${result} ${dep2} PARENT_SCOPE)
        return()
    endif()

    # Exception for packages started with rc110_, because they can appear in the same workspace
    if (dep MATCHES "^rc110_")
        to_deb_name(dep3 ros-$ENV{ROS_DISTRO}-${dep})
        set(${result} ${dep3} PARENT_SCOPE)
        return()
    endif()

    set(${result} "" PARENT_SCOPE)
    if(CATKIN_BUILD_BINARY_PACKAGE)
        message(WARNING "Couldn't convert dependency to deb name: ${dep}")
    endif()
endfunction()

if(NOT CPACK_PACKAGE_NAME)
    set(CPACK_PACKAGE_NAME ros-$ENV{ROS_DISTRO}-${PROJECT_NAME})
endif()
to_deb_name(CPACK_PACKAGE_NAME ${CPACK_PACKAGE_NAME})

set(CPACK_GENERATOR DEB)
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "ZMP Inc. <info@zmp.co.jp>")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PROJECT_NAME}")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_DEBIAN_PACKAGE_RELEASE "")
set(CPACK_PACKAGING_INSTALL_PREFIX /opt/ros/$ENV{ROS_DISTRO})

foreach(dependency IN LISTS ${PROJECT_NAME}_EXEC_DEPENDS)
    ros_to_deb_name(deb ${dependency})

    if (NOT ${deb} STREQUAL "")
        if (CPACK_DEBIAN_PACKAGE_DEPENDS)
            set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, ")
        endif()
        set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}${deb}")
    endif()
endforeach()

include(CPack)