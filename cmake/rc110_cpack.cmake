##
# CPack modification for ROS packages. Usage is similar to CPack, except the following.
#
# set(RC110_DEP.<ros_name> <deb-name>)  # allows to map ros dependency to debian one
# include(<>/rc110_cpack.cmake)
##

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
# Convert ros dependency name to debian package names.
#
function(ros_to_deb_names result dep)
    # Some packages can be found with rosdep resolve.
    execute_process(
            COMMAND rosdep resolve ${dep}
            COMMAND sed -n 2p  # 2nd line
            OUTPUT_VARIABLE dep_ros
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULTS_VARIABLE results
    )
    list(GET results 0 rosdep_result)
    if(rosdep_result EQUAL 0)
        set(${result} ${dep_ros} PARENT_SCOPE)
        return()
    endif()

    # Additional dependencies that rosdep failed to find.
    if(RC110_DEP.${dep})
        set(${result} ${RC110_DEP.${dep}} PARENT_SCOPE)
        return()
    endif()

    # Additional dependencies started with rc110_, because they can appear in the same workspace.
    if (dep MATCHES "^rc110")
        to_deb_name(dep_local ros-$ENV{ROS_DISTRO}-${dep})
        set(${result} ${dep_local} PARENT_SCOPE)
        return()
    endif()

    set(${result} "" PARENT_SCOPE)
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
set(CPACK_PACKAGING_INSTALL_PREFIX /opt/ros/$ENV{ROS_DISTRO})

foreach(dependency IN LISTS ${PROJECT_NAME}_EXEC_DEPENDS)
    ros_to_deb_names(debs ${dependency})
    if (NOT debs)
        message(WARNING "Couldn't convert dependency to deb name: ${dependency}")
        continue()
    endif()

    string(REPLACE " " ";" debs_list ${debs})
    foreach(deb ${debs_list})
        if (CPACK_DEBIAN_PACKAGE_DEPENDS)
            set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, ")
        endif()
        set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}${deb}")
    endforeach()
endforeach()

include(CPack)