# CMakeList.txt : CMake project for Jaq, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.15)

project ("Jaq" VERSION "0.2.0")

# Add source to this project's executable.
#add_executable (Jaq "Jaq.cpp" "Jaq.h")

set (CMAKE_CXX_STANDARD 14)

if (${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")

	# MacOS
	set (CMAKE_OSX_ARCHITECTURES=x86_64)
	set (CMAKE_OSX_DEPLOYMENT_TARGET 10.9)
    set (CMAKE_XCODE_ATTRIBUTE_CODE_SIGNING_REQUIRED "NO")
    set (CMAKE_XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "")
	#set (CMAKE_PREFIX_PATH /usr/local)

elseif (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

	# Linux
	#set (CMAKE_PREFIX_PATH /usr/local)
	
elseif (${CMAKE_SYSTEM_NAME} STREQUAL "Windows")

	# MSWindows
	cmake_policy(SET CMP0091 NEW)
	set (CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

else ()

	# not supported
	message ("Jaq is not configured to build on this platform: " ${CMAKE_SYSTEM_NAME} ".")

endif ()

#
# ---- Jaq Source Files -------------------------------------------------------------
#

set (sources
	Jaq.cpp
	Jaq.h
	JaqUI.cxx
	JaqUI.h
	JaqUI.fl
	SCServo.cpp
	SCServo.h
	"JQServo.cpp" 
	"JQServo.h" 
	"JQLeg.cpp" 
	"JQLeg.h" 
	"JQServoController.cpp" 
	"JQServoController.h" 
	"JQBot.cpp" 
	"JQBot.h" 
	"JQMisc.cpp" 
	"JQMisc.h"
	)

#set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/app/Version.h PROPERTIES GENERATED TRUE)

string(TIMESTAMP COMPILE_TIME_YYYY "%Y")
string(TIMESTAMP COMPILE_TIME_MM "%m")
string(TIMESTAMP COMPILE_TIME_DD "%d")

#configure_file(
#	${CMAKE_CURRENT_SOURCE_DIR}/app/Version.h.in
#	${CMAKE_CURRENT_SOURCE_DIR}/app/Version.h
#)


macro (build_with_fluid name dir)
	add_custom_command (
		WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${dir}"
		COMMAND ${FLTK_FLUID_EXECUTABLE} -c ${name}.fl
		DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}/${name}.fl"
		#DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/app/Version.h"
		OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/${dir}/${name}.cpp"
		OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/${dir}/${name}.h"
	)
	set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/${dir}/${name}.cpp PROPERTIES GENERATED TRUE)
	set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/${dir}/${name}.h PROPERTIES GENERATED TRUE)
endmacro ()


if (${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")

	# configure to use FLTK as the GUI
	set(FLTK_SKIP_OPENGL true)
	set(FLTK_FLUID_EXECUTABLE /usr/local/bin/fluid)
	find_package(FLTK REQUIRED NO_MODULE)

	build_with_fluid(JaqUI .)

	# additional source code
	list (APPEND sources
		# calling ObjectiveC native functions (very limited support in this version of EInsten)
		SCSerial.darwin.cpp
		SCSerial.darwin.h
	)
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)
	source_group(TREE "${CMAKE_SOURCE_DIR}" PREFIX "Sources" FILES ${sources})

	# take care of the app icon
	#set (MACOSX_BUNDLE_ICON_FILE Jaq.icns)
	#set_source_files_properties(Resources/macOS/${MACOSX_BUNDLE_ICON_FILE} PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")

	# create the application
	add_executable(Jaq MACOSX_BUNDLE
		${sources}
	)

	# how to compile and link
	target_compile_options(Jaq PUBLIC -Wall)
	target_compile_definitions(Jaq PRIVATE TARGET_UI_FLTK=1 NO_PORT_AUDIO NO_X11 TARGET_OS_OPENSTEP=1 TARGET_OS_MAC=1 NS_BLOCK_ASSERTIONS=1)
	target_link_libraries(Jaq
		# /usr/local/lib/libfltk.a
		fltk
		pthread
		"-framework AppKit"
		"-framework Cocoa"
	)
	set_target_properties(Jaq PROPERTIES
		#MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/Resources/macOS/Info.plist.in"
	)
    set_target_properties(Jaq PROPERTIES
        #CPACK_BUNDLE_APPLE_ENTITLEMENTS "${CMAKE_CURRENT_SOURCE_DIR}/Resources/macOS/EntiXXXtlements.plist"
    )

elseif (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

	# configure to use FLTK as the GUI
	set(FLTK_SKIP_OPENGL true)
	set(FLTK_FLUID_EXECUTABLE /usr/local/bin/fluid)
	find_package(FLTK REQUIRED NO_MODULE)

	build_with_fluid(JaqUI .)

	# additional source code
	list (APPEND sources
		SCSerial.linux.cpp
		SCSerial.linux.h
			)
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)
	source_group(TREE "${CMAKE_SOURCE_DIR}" PREFIX "Sources" FILES ${sources})

	# create the application
	add_executable(Jaq ${sources})

	# how to compile and link
	target_compile_options(Jaq PUBLIC -Wall -Wno-multichar)
	target_compile_definitions(Jaq PRIVATE TARGET_UI_FLTK=1 TARGET_OS_LINUX=1)
	target_link_libraries(Jaq
		fltk
	)

elseif (WIN32)

	# configure to use FLTK as the GUI
	set(FLTK_DIR "C:/Users/micro/dev/fltk-1.4 for Jaq.git/out/install/x64-Release/CMake")
	# set(FLTK_FLUID_EXECUTABLE "C:/Program Files/FLTK/bin/fluid.exe")
	set(FLTK_SKIP_OPENGL true)
	find_package(FLTK REQUIRED NO_MODULE)

	build_with_fluid(JaqUI .)

	# additional source code
	list (APPEND sources
		"SCServo.win32.cpp"
		"SCServo.win32.h"
	)
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)
	#source_group(TREE "${CMAKE_SOURCE_DIR}" PREFIX "Sources" FILES ${sources} Resources/MSWindows/Jaq.rc.in)

	# Create a resources file for Windows
	#configure_file(
	#	Resources/MSWindows/Jaq.rc.in
	#	Jaq.rc
	#)

	# create the application
	add_executable(Jaq WIN32
		${sources} ${data}
		#${CMAKE_CURRENT_BINARY_DIR}/Jaq.rc
	)

	# how to compile and link
	# warning 26812: yes, enum classes are better than enums, I know
	target_compile_options(Jaq PUBLIC "/bigobj" "/wd26812")
	target_compile_definitions(Jaq PRIVATE TARGET_UI_FLTK=1 TARGET_OS_WIN32=1 WIN32_LEAN_AND_MEAN=1 _CRT_SECURE_NO_WARNINGS=1)
	target_link_libraries(Jaq
		# this vvv always includes the debug version, even in release mode
		fltk
		# alternatively, this loads the correct library but needs additional libs that would be included automatically in the line above
		#"${FLTK_BASE_LIBRARY_RELEASE}" # FIXME: or it will select the debug version even in release mode!
		#comctl32
		#ws2_32
		# end of alternative
		#ucrt
		winmm
	)

endif ()


target_include_directories(
	Jaq PUBLIC
	${CMAKE_SOURCE_DIR}
	${FLTK_INCLUDE_DIRS}
)


# nice code to output every know variable in this CMake session
#get_cmake_property(_variableNames VARIABLES)
#list (SORT _variableNames)
#foreach (_variableName ${_variableNames})
#	message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()


#
# --- MessagePad.org distribution system ---------------------------------------
# This has not yet been adapted to JaQ. Ignore.
#

set (EINSTEIN_DIST_DIR "Sites/messagepad.org/htdocs/Downloads/Einstein")

if (${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")

	# MacOS
	add_custom_target(
		dist
		COMMAND ${CMAKE_COMMAND} -E tar "cfv" "/Users/matt/${EINSTEIN_DIST_DIR}/MacOS/Einstein.MacOS.${CMAKE_PROJECT_VERSION}.zip" --format=zip -- "Einstein.app"
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	)
    add_dependencies(dist Einstein)

elseif (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

	# Linux
	add_custom_target(
		dist
		COMMAND ${CMAKE_COMMAND} -E tar "cfv" "/media/psf/Home/${EINSTEIN_DIST_DIR}/Linux/Einstein.Linux.${CMAKE_PROJECT_VERSION}.zip" --format=zip -- "Einstein"
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	)
    add_dependencies(dist Einstein)

elseif (${CMAKE_SYSTEM_NAME} STREQUAL "Windows")

	# MSWindows
	add_custom_target(
		dist
		COMMAND ${CMAKE_COMMAND} -E tar "cfv" "z:/${EINSTEIN_DIST_DIR}/MSWindows/Einstein.Windows.${CMAKE_PROJECT_VERSION}.zip" --format=zip -- "Einstein.exe"
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	)
    add_dependencies(dist Jaq)

else ()

	# not supported
	message ("Einstein is not configured to distribute for this platform: " ${CMAKE_SYSTEM_NAME} ".")

endif ()

