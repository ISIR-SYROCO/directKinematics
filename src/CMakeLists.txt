
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libdirectKinematics-<target>.so
# and installs in the directory lib/orocos/directKinematics/
#
orocos_component(directKinematics directKinematics-component.hpp directKinematics-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/directKinematics/ directory
#
orocos_install_headers(directKinematics-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libdirectKinematics-plugin-<target>.so
# and installs in the directory lib/orocos/directKinematics/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(directKinematics-plugin directKinematics-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libdirectKinematics-service-<target>.so
# and installs in the directory lib/orocos/directKinematics/plugins/
#
orocos_service(directKinematics-service directKinematics-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library libdirectKinematics-types-<target>.so
# and installs in the directory lib/orocos/@target@/directKinematics/types/
#
# The header will go in include/orocos/directKinematics/types/directKinematics/directKinematics-types.hpp
# So you can #include <directKinematics/directKinematics-types.hpp>
#
orocos_typegen_headers(include/directKinematics/directKinematics-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement in your toplevel CMakeFile.txt !


#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
orocos_library(support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


