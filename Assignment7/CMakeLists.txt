cmake_minimum_required(VERSION 3.10)
project(RayTracing3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

add_executable(RayTracing3 main.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
        Scene.hpp Light.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
        Renderer.cpp Renderer.hpp)

file(COPY ${PROJECT_SOURCE_DIR}/../opencv/bin/opencv_world452d.dll DESTINATION ${PROJECT_BINARY_DIR}/Debug/)
file(COPY ${PROJECT_SOURCE_DIR}/models/ DESTINATION ${PROJECT_BINARY_DIR}/../models/)