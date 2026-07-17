cmake_minimum_required(VERSION 3.19)

project(Newell LANGUAGES CXX)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Widgets OpenGL OpenGLWidgets Concurrent)
find_package(Eigen3 QUIET CONFIG)

option(NEWELL_ENABLE_CGAL "Enable CGAL-backed reconstruction methods" ON)
if (NEWELL_ENABLE_CGAL)
    find_package(CGAL CONFIG QUIET)
endif()

option(NEWELL_ENABLE_POISSON "Enable a screened octree Poisson backend" OFF)
if (NEWELL_ENABLE_POISSON)
    message(FATAL_ERROR
        "NEWELL_ENABLE_POISSON was requested, but no screened octree Poisson provider is configured")
endif()

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

set(NEWELL_MACOS_ICON "${CMAKE_CURRENT_SOURCE_DIR}/assets/app_icon/macos/Newell.icns")
set(NEWELL_RUNTIME_ICONS_QRC "${CMAKE_CURRENT_SOURCE_DIR}/assets/app_icon/newell_icons.qrc")

if (APPLE)
    set(MACOSX_BUNDLE_ICON_FILE "Newell.icns")

    set_source_files_properties(
        ${NEWELL_MACOS_ICON}
        PROPERTIES
            MACOSX_PACKAGE_LOCATION "Resources"
    )
endif()

add_library(NewellCore STATIC
    src/controller/ApplicationState.cpp
    src/controller/ApplicationState.h
    src/controller/PreprocessingController.cpp
    src/controller/PreprocessingController.h
    src/controller/SurfaceReconstructionController.cpp
    src/controller/SurfaceReconstructionController.h
    src/model/geometry/BoundingBox3d.cpp
    src/model/geometry/BoundingBox3d.h
    src/model/geometry/GeometryDocument.cpp
    src/model/geometry/GeometryDocument.h
    src/model/geometry/NormalField.cpp
    src/model/geometry/NormalField.h
    src/model/geometry/Point3d.h
    src/model/geometry/PointCloud.cpp
    src/model/geometry/PointCloud.h
    src/model/geometry/TriangleMesh.cpp
    src/model/geometry/TriangleMesh.h
    src/model/io/PlyPointCloudImporter.cpp
    src/model/io/PlyPointCloudImporter.h
    src/model/preprocessing/DuplicatePointSelection.cpp
    src/model/preprocessing/DuplicatePointSelection.h
    src/model/preprocessing/InvalidPointSelection.cpp
    src/model/preprocessing/InvalidPointSelection.h
    src/model/processing/normals/INormalEstimator.h
    src/model/processing/normals/IFieldNormalGenerator.h
    src/model/processing/normals/INormalOrienter.h
    src/model/processing/normals/GraphNormalOrienter.cpp
    src/model/processing/normals/GraphNormalOrienter.h
    src/model/processing/normals/ImplicitFieldGradientNormalGenerator.cpp
    src/model/processing/normals/ImplicitFieldGradientNormalGenerator.h
    src/model/processing/normals/LocalSurfaceAnalysis.cpp
    src/model/processing/normals/LocalSurfaceAnalysis.h
    src/model/processing/normals/MultiScalePcaNormalEstimator.cpp
    src/model/processing/normals/MultiScalePcaNormalEstimator.h
    src/model/processing/normals/NormalMethodRegistry.cpp
    src/model/processing/normals/NormalMethodRegistry.h
    src/model/processing/normals/NormalProcessingTypes.h
    src/model/processing/normals/PcaKnnNormalEstimator.cpp
    src/model/processing/normals/PcaKnnNormalEstimator.h
    src/model/processing/normals/PcaRadiusNormalEstimator.cpp
    src/model/processing/normals/PcaRadiusNormalEstimator.h
    src/model/processing/normals/QuadraticSurfaceNormalEstimator.cpp
    src/model/processing/normals/QuadraticSurfaceNormalEstimator.h
    src/model/processing/normals/ScalarGridGradientNormalGenerator.cpp
    src/model/processing/normals/ScalarGridGradientNormalGenerator.h
    src/model/processing/normals/ViewpointNormalOrienter.cpp
    src/model/processing/normals/ViewpointNormalOrienter.h
    src/model/processing/reconstruction/BallPivotingReconstructor.cpp
    src/model/processing/reconstruction/BallPivotingReconstructor.h
    src/model/processing/reconstruction/AlphaShapeReconstructor.cpp
    src/model/processing/reconstruction/AlphaShapeReconstructor.h
    src/model/processing/reconstruction/Delaunay25DReconstructor.cpp
    src/model/processing/reconstruction/Delaunay25DReconstructor.h
    src/model/processing/reconstruction/GreedyProjectionReconstructor.cpp
    src/model/processing/reconstruction/GreedyProjectionReconstructor.h
    src/model/processing/reconstruction/ISurfaceReconstructor.h
    src/model/processing/reconstruction/MarchingCubesReconstructor.cpp
    src/model/processing/reconstruction/MarchingCubesReconstructor.h
    src/model/processing/reconstruction/RbfReconstructor.cpp
    src/model/processing/reconstruction/RbfReconstructor.h
    src/model/processing/reconstruction/ReconstructionTypes.h
    src/model/processing/reconstruction/ScreenedPoissonReconstructor.cpp
    src/model/processing/reconstruction/ScreenedPoissonReconstructor.h
    src/model/processing/reconstruction/SurfaceReconstructorRegistry.cpp
    src/model/processing/reconstruction/SurfaceReconstructorRegistry.h
    src/model/processing/reconstruction/VoxelReconstructor.cpp
    src/model/processing/reconstruction/VoxelReconstructor.h
    src/model/processing/reconstruction/common/MeshValidation.cpp
    src/model/processing/reconstruction/common/MeshValidation.h
    src/model/processing/reconstruction/common/ProjectionPlaneUtilities.cpp
    src/model/processing/reconstruction/common/ProjectionPlaneUtilities.h
    src/model/processing/reconstruction/common/ScalarGrid3D.cpp
    src/model/processing/reconstruction/common/ScalarGrid3D.h
    src/model/processing/spatial/KDTree.cpp
    src/model/processing/spatial/KDTree.h
    src/model/processing/spatial/NeighbourSearch.h
    src/model/processing/common/CancellationToken.h
    src/model/processing/common/ProcessingProgress.cpp
    src/model/processing/common/ProcessingProgress.h
    src/model/processing/common/ProgressReporter.cpp
    src/model/processing/common/ProgressReporter.h
    src/model/processing/preprocessing/SelectionAlgorithms.cpp
    src/model/processing/preprocessing/SelectionAlgorithms.h
    src/model/pipeline/PreprocessingPipeline.cpp
    src/model/pipeline/PreprocessingPipeline.h
    src/model/pipeline/ReconstructionPipeline.cpp
    src/model/pipeline/ReconstructionPipeline.h
)

target_include_directories(NewellCore
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(NewellCore
    PUBLIC
        Eigen3::Eigen
)

if (CGAL_FOUND)
    target_link_libraries(NewellCore PUBLIC CGAL::CGAL)
    target_compile_definitions(NewellCore PUBLIC NEWELL_HAS_CGAL=1)
else()
    target_compile_definitions(NewellCore PUBLIC NEWELL_HAS_CGAL=0)
    message(STATUS "CGAL not found: Delaunay 2.5D and Alpha Shapes will be unavailable")
endif()

qt_add_executable(Newell
    WIN32
    MACOSX_BUNDLE
    ${NEWELL_RUNTIME_ICONS_QRC}
    src/app/main.cpp
    src/view/MainWindow.cpp
    src/view/MainWindow.h
    src/view/MainWindow.ui
    src/view/MethodSelectionDialog.cpp
    src/view/MethodSelectionDialog.h
    src/view/FormattedDoubleSpinBox.cpp
    src/view/FormattedDoubleSpinBox.h
    src/view/FloatingWorkflowMenu.cpp
    src/view/FloatingWorkflowMenu.h
    src/view/PreprocessingSideMenu.cpp
    src/view/PreprocessingSideMenu.h
    src/view/PointCloudViewport.cpp
    src/view/PointCloudViewport.h
    src/view/ProcessingProgressWidget.cpp
    src/view/ProcessingProgressWidget.h
)

if (APPLE)
    target_sources(Newell PRIVATE ${NEWELL_MACOS_ICON})

    set_target_properties(Newell PROPERTIES
        MACOSX_BUNDLE TRUE
        MACOSX_BUNDLE_ICON_FILE "Newell.icns"
    )
endif()

target_include_directories(Newell
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(Newell
    PRIVATE
        NewellCore
        Qt::Core
        Qt::Concurrent
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

add_executable(NewellReconstructionTests
    tests/ReconstructionTests.cpp
)

target_link_libraries(NewellReconstructionTests
    PRIVATE
        NewellCore
)

add_test(NAME NewellReconstructionTests COMMAND NewellReconstructionTests)

add_executable(NewellNormalProcessingTests
    tests/NormalProcessingTests.cpp
)

target_link_libraries(NewellNormalProcessingTests
    PRIVATE
        NewellCore
)

add_test(NAME NewellNormalProcessingTests COMMAND NewellNormalProcessingTests)

add_executable(NewellProcessingTests
    tests/ProcessingTests.cpp
)

target_link_libraries(NewellProcessingTests
    PRIVATE
        NewellCore
)

add_test(NAME NewellProcessingTests COMMAND NewellProcessingTests)

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
