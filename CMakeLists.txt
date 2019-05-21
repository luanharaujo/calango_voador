
project(calango_voador)


include_directories(./libs/librobotcontrol/include)
set(LIBCONTROLLIB ${CMAKE_CURRENT_SOURCE_DIR}/libs/librobotcontrol/lib/librobotcontrol.so.1.0.4)

add_executable(main.out 
                src/main.c)
target_link_libraries(main.out ${LIBCONTROLLIB} m)
                
