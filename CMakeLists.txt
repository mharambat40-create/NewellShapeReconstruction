cmake_minimum_required(VERSION 3.19)

project(Newell LANGUAGES CXX)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Widgets OpenGL OpenGLWidgets)
find_package(Eigen3 QUIET CONFIG)

if (NOT Eigen3_FOUND)
    find_path(EIGEN3_INCLUDE_DIR
        NAMES Eigen/Core
        PATH_SUFFIXES eigen3
        PATHS
            /opt/homebrew/include
            /usr/local/include
            /opt/homebrew/opt/eigen/include
            /usr/local/opt/eigen/include
    )

    if (EIGEN3_INCLUDE_DIR)
        add_library(Eigen3::Eigen INTERFACE IMPORTED)
        set_target_properties(Eigen3::Eigen PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}"
        )
        set(Eigen3_FOUND TRUE)
    else()
        message(FATAL_ERROR
            "Eigen3 was not found. Install it on macOS with 'brew install eigen' "
            "and then rerun CMake.")
    endif()
endif()

qt_standard_project_setup()

add_library(NewellCore STATIC
    src/controller/ApplicationState.cpp
    src/controller/ApplicationState.h
    src/model/geometry/BoundingBox3d.cpp
    src/model/geometry/BoundingBox3d.h
    src/model/geometry/GeometryDocument.cpp
    src/model/geometry/GeometryDocument.h
    src/model/geometry/Point3d.h
    src/model/geometry/PointCloud.cpp
    src/model/geometry/PointCloud.h
    src/model/io/PlyPointCloudImporter.cpp
    src/model/io/PlyPointCloudImporter.h
)

target_include_directories(NewellCore
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(NewellCore
    PUBLIC
        Eigen3::Eigen
)

qt_add_executable(Newell
    WIN32
    MACOSX_BUNDLE
    src/app/main.cpp
    src/view/MainWindow.cpp
    src/view/MainWindow.h
    src/view/MainWindow.ui
    src/view/FloatingWorkflowMenu.cpp
    src/view/FloatingWorkflowMenu.h
    src/view/PointCloudViewport.cpp
    src/view/PointCloudViewport.h
)

target_include_directories(Newell
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(Newell
    PRIVATE
        NewellCore
        Qt::Core
        Qt::OpenGL
        Qt::OpenGLWidgets
        Qt::Widgets
)

enable_testing()

add_executable(NewellTests
    tests/ModelImportTests.cpp
)

target_link_libraries(NewellTests
    PRIVATE
        NewellCore
)

add_test(NAME NewellTests COMMAND NewellTests)

install(TARGETS Newell
    BUNDLE DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

qt_generate_deploy_app_script(
    TARGET Newell
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})
