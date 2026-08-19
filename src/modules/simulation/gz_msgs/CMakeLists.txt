############################################################################
#
#   Copyright (c) 2025 PX4 Development Team. All rights reserved.
#
############################################################################

# Find required packages
find_package(Protobuf)

if (Protobuf_FOUND)
    # Generate protobuf messages
    file(GLOB MSGS_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
    PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${MSGS_PROTOS})

    # Create library
    add_library(px4_gz_msgs STATIC
        ${PROTO_SRCS}
        ${PROTO_HDRS}
    )

    target_include_directories(px4_gz_msgs
        PUBLIC
            ${CMAKE_CURRENT_BINARY_DIR}
            ${Protobuf_INCLUDE_DIRS}
    )
    target_link_libraries(px4_gz_msgs PUBLIC ${PROTOBUF_LIBRARIES})
    if (Protobuf_VERSION VERSION_LESS "3.8")
        target_compile_options(px4_gz_msgs PRIVATE -Wno-error=float-equal)
    endif()

    # PX4 builds the whole project at the global CMAKE_CXX_STANDARD (14), which
    # NuttX/embedded targets require. On macOS, Homebrew's protobuf pulls in a
    # recent abseil that hard-requires C++17 in its own headers
    # (absl/base/policy_checks.h: "C++ versions less than C++17 are not
    # supported."). Scoped to this one target only -- does not change the
    # standard used anywhere else in the build.
    set_target_properties(px4_gz_msgs PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
    )

    # PX4's global -Wdouble-promotion + -Werror flags apply to this target
    # too, but fire on a float->double promotion inside Homebrew's own
    # abseil header (absl/strings/str_cat.h), which we don't control and
    # can't fix. Matches the -Wno-error=float-equal precedent above.
    target_compile_options(px4_gz_msgs PRIVATE -Wno-error=double-promotion)

    # Export the binary dir for other modules
    set(PX4_GZ_MSGS_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
endif()
