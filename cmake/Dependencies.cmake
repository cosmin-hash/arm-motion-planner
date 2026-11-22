# Third-party dependencies, fetched at configure time so the build is self-contained.
include(FetchContent)

# --- Eigen (header-only) ---------------------------------------------------
# We only need the headers. SOURCE_SUBDIR points at a directory that does not exist,
# so FetchContent downloads Eigen but does NOT run Eigen's own (slow, test-heavy) CMakeLists.
FetchContent_Declare(eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    SOURCE_SUBDIR do-not-configure
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(eigen)

if(NOT TARGET Eigen3::Eigen)
    add_library(amp_eigen INTERFACE)
    # SYSTEM: suppress warnings from Eigen headers so /WX and -Werror only apply to our code.
    target_include_directories(amp_eigen SYSTEM INTERFACE ${eigen_SOURCE_DIR})
    # Eigen's fixed-size vectorisation alignment rules are a classic source of subtle bugs
    # when objects are stored in std::vector. We only use small dynamic and 2D types, and
    # disabling static alignment keeps the code simple and portable.
    target_compile_definitions(amp_eigen INTERFACE EIGEN_MAX_STATIC_ALIGN_BYTES=0)
    add_library(Eigen3::Eigen ALIAS amp_eigen)
endif()

# --- GoogleTest ------------------------------------------------------------
if(AMP_BUILD_TESTS)
    FetchContent_Declare(googletest
        URL https://github.com/google/googletest/releases/download/v1.15.2/googletest-1.15.2.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    # Use the same (dynamic) CRT as the rest of the project on MSVC.
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

# --- nlohmann/json (scenario files) ------------------------------------------
FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(JSON_BuildTests OFF CACHE INTERNAL "")
FetchContent_MakeAvailable(nlohmann_json)
