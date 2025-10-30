cmake_minimum_required(VERSION 3.28)

project(host_code
	VERSION 0.0.0
	DESCRIPTION "todo"
	LANGUAGES CXX
)

# --- チェック ---
# if(NOT CMAKE_CXX_COMPILER)
# 	message(FATAL_ERROR "No C++ compiler specified. Please set CMAKE_CXX_COMPILER explicitly (e.g., cmake -DCMAKE_CXX_COMPILER=clang++ ..).")
# endif()

if(NOT (CMAKE_GENERATOR MATCHES "Ninja"))
	message(FATAL_ERROR
		"This project requires the Ninja. "
		"Your current generator is '${CMAKE_GENERATOR}'."
	)
endif()

if(NOT DEFINED CMAKE_INSTALL_PREFIX OR CMAKE_INSTALL_PREFIX STREQUAL "/usr/local")
	message(WARNING
		"CMAKE_INSTALL_PREFIX should be set explicitly.\n"
		"Please configure with `-DCMAKE_INSTALL_PREFIX=<path>`.\n"
		"set CMAKE_INSTALL_PREFIX=./install"
	)
	unset(CMAKE_INSTALL_PREFIX CACHE)
	set(CMAKE_INSTALL_PREFIX ./install CACHE STRING "CMAKE_INSTALL_PREFIX")
endif()

if(NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
	message(WARNING
		"CMAKE_BUILD_TYPE should be set explicitly.\n"
		"Please configure with `-DCMAKE_BUILD_TYPE=<Debug or Release>`.\n"
		"set CMAKE_BUILD_TYPE=Debug"
	)
	unset(CMAKE_BUILD_TYPE CACHE)
	set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build.")
endif()


# --- ビルド設定 ---
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_STATIC_LIB "build static_lib" ON)
option(BUILD_SHARED_LIB "build shared_lib" OFF)
option(BUILD_MODULE_LIB "build module_lib" OFF)
option(BUILD_DEVS "Build devs" OFF)
set(RELEASE_OPTIMIZE -O2 CACHE STRING "The compiler option for optimization, e.g. -O2 or -Os")


# --- find_package(dev用のものはdev.cmake内で) ---

# --- 依存を指定 ---
## libとbinの依存が共通 -> binはあくまでlibに従うもの、libの使わないものを使いたいならプロジェクトを分けるべき
add_library(${PROJECT_NAME}_dep_private INTERFACE EXCLUDE_FROM_ALL)
target_link_libraries(${PROJECT_NAME}_dep_private
	INTERFACE
		# add private dependencies (s.t. no header depends but any source depends)
)
target_compile_definitions(${PROJECT_NAME}_dep_private
	INTERFACE
		# add macro definition if need
)
target_include_directories(${PROJECT_NAME}_dep_private
	INTERFACE
	../spawn_coro/include/
	../cobs_bytestuff/include/
)

add_library(${PROJECT_NAME}_dep_export INTERFACE EXCLUDE_FROM_ALL)
target_link_libraries(${PROJECT_NAME}_dep_export
	INTERFACE
		# add exported dependencies (s.t. any header depends)
)
target_include_directories(${PROJECT_NAME}_dep_export
	INTERFACE
)


# --- 各種ターゲットを作成 ---
set(install_target_list "")
include(cmake/common.cmake)
include(cmake/lib.cmake)
include(cmake/bin.cmake)

if(BUILD_DEVS)
	include(CTest)
	include(FetchContent)

	# --- find package ---
	# find_package(foo REQUIRED)

	FetchContent_Declare(doctest
		GIT_REPOSITORY https://github.com/doctest/doctest.git
	)
	FetchContent_Populate(doctest)  # doctestのincludeパスが多すぎるので...

	# --- 依存の指定 ---
	add_library(${PROJECT_NAME}_dev_dep INTERFACE EXCLUDE_FROM_ALL)
	target_link_libraries(${PROJECT_NAME}_dev_dep
		INTERFACE
			# add dev dependencies
	)
	target_include_directories(${PROJECT_NAME}_dev_dep
		INTERFACE
			${doctest_SOURCE_DIR}/doctest/  # doctestのincludeパスが多すぎるので...
	)

	set(install_example_target_list "")
	include(cmake/dev.cmake)
endif()


# --- 個別に追加したいものがあればここで ---
# target_compile_definitions(${PROJECT_NAME}_bin_foo
# 	PRIVATE
# 		FOO
# )

# --- インストール設定 ---
include(GNUInstallDirs)

message("install_target_list: ${install_target_list}")
install(TARGETS
	${install_target_list}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/${PROJECT_NAME}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

if(BUILD_DEVS)
	install(TARGETS
		${install_example_target_list}
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/${PROJECT_NAME}/examples/
	)
endif()