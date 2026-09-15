cmake_minimum_required(VERSION 3.16)
project(test_raylib LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(raylib)

add_executable(test
    test.cpp
    phys/world.cpp
    phys/collision.cpp
)
target_include_directories(test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test PRIVATE raylib)

add_executable(game
    game.cpp
    phys/world.cpp
    phys/collision.cpp
)
target_include_directories(game PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(game PRIVATE raylib)

# ソースは UTF-8。MSVC は既定でロケールのコードページ（日本語環境では CP932）
# として読むため、指定しないと日本語コメント末尾が次の行を巻き込んで壊れる。
foreach(tgt test game)
    target_compile_options(${tgt} PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)
endforeach()
