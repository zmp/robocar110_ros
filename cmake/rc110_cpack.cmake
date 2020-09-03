if(NOT DEFINED ENV{ROS_DISTRO})
    message(FATAL_ERROR "ROS_DISTRO is not defined. Please, source ros.")
endif()

function(to_deb_name result name)
    string(REGEX REPLACE "_" "-" name ${name})
    string(TOLOWER "${name}" name)
    set(${result} ${name} PARENT_SCOPE)
endfunction()

if(NOT CPACK_PACKAGE_NAME)
    set(CPACK_PACKAGE_NAME ${CMAKE_PROJECT_NAME})
endif()
to_deb_name(CPACK_PACKAGE_NAME ${CPACK_PACKAGE_NAME})

set(CPACK_GENERATOR DEB CACHE STRING "")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT CACHE STRING "")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "ZMP Inc. <info@zmp.co.jp>" CACHE STRING "")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${CMAKE_PROJECT_NAME}" CACHE STRING "")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}" CACHE STRING "")
set(CPACK_DEBIAN_PACKAGE_RELEASE "" CACHE STRING "")
set(CPACK_PACKAGING_INSTALL_PREFIX /opt/ros/$ENV{ROS_DISTRO} CACHE STRING "")

foreach(dependency IN LISTS catkin_exec_depends)
    if (CPACK_DEBIAN_PACKAGE_DEPENDS)
        set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, ")
    endif()

    to_deb_name(deb ros-$ENV{ROS_DISTRO}-${dependency})
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}${deb}")
endforeach()

include(CPack)