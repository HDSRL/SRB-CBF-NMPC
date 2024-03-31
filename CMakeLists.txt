cmake_minimum_required(VERSION 3.5)
project(A1_Robot VERSION 1.1.1)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3 -fopenmp -std=c++14 -ggdb")
set(OGRE_DIR "/home/basit/workspace/raisimLib/build/lib/OGRE/cmake/")

set(LL_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# ===== Path to RaiSim and RaiSimOgre ===== #
if($ENV{USER} STREQUAL "basit")
    find_package(raisim REQUIRED HINTS "/home/basit/workspace/raisimLib/build/lib/cmake/raisim")
    find_package(raisim REQUIRED HINTS "/home/basit/workspace/raisimLib/raisim/linux/lib/cmake/raisim")
    find_package(raisimOgre REQUIRED HINTS "/home/basit/workspace/raisimOgre/build/cmake")
else()
    # find_package(raisim REQUIRED PATHS "/home/basit/workspace/raisimLib/cmake/raisim")
    # find_package(raisimOgre CONFIG PATHS "/home/basit/workspace/raisimOgre/cmake")
endif()


find_package(ifopt)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(Boost REQUIRED COMPONENTS timer system thread)
find_library(LCM lcm HINTS /usr/local/lib)

if (raisimOgre_FOUND)
    include_directories(PUBLIC ${LL_DIR}/raisimOgre_include)
    file(COPY ${LL_DIR}/rsc DESTINATION ${CMAKE_BINARY_DIR})
endif()

include_directories(${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
set(UT_INC ${CMAKE_CURRENT_SOURCE_DIR}/unitree_legged_sdk/include)
set(UT_LINK ${CMAKE_CURRENT_SOURCE_DIR}/unitree_legged_sdk/lib)
set(EXTRA_LIBS -pthread ${UT_LINK}/libunitree_legged_sdk_amd64.so)

# ===== Dynamics Library ===== #

# Settings:
set(COMPILE_CORIOLIS FALSE) # Option to compile the coriolis terms. Not implemented in code though.

set(dyn_srcs 
    "${LL_DIR}/dynamics/A1_New_wFoot/A1_D_B_G.cpp"
    "${LL_DIR}/dynamics/A1_New_wFoot/A1_FK.cpp"
    "${LL_DIR}/dynamics/A1_New_wFoot/A1_FK.cpp"
    "${LL_DIR}/dynamics/A1_New_wFoot/A1_Jaco.cpp"
    "${LL_DIR}/dynamics/A1_New_wFoot/A1_Jaco_dot.cpp")
if (COMPILE_CORIOLIS)
    set(dyn_srcs ${dyn_srcs} "${LL_DIR}/dynamics/A1_New_wFoot/A1_C.cpp")
endif()
add_library(dyn_lib ${dyn_srcs})
target_include_directories(dyn_lib PUBLIC
    "${LL_DIR}/dynamics"
    "${LL_DIR}/dynamics/A1_New_wFoot"
)

# ===== Controller Library ===== #
add_library(ctrl_lib
    "${LL_DIR}/src/Parameters.cpp"
    "${LL_DIR}/src/RobotModel.cpp"
    "${LL_DIR}/src/LocoWrapper.cpp"
    "${LL_DIR}/src/LowLevelCtrl.cpp"
    "${LL_DIR}/src/VirtualConstraints.cpp"
    "${LL_DIR}/src/ContactEst.cpp"
    "${LL_DIR}/src/MotionPlanner.cpp"
    "${LL_DIR}/src/DataLog.cpp"
    "${LL_DIR}/src/MPC_dist.cpp"
)
target_include_directories(ctrl_lib PUBLIC 
    ${LL_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    ${LL_DIR}/optimization/iSWIFT/include
    ${LL_DIR}/optimization/iSWIFT/ldl/include
    ${LL_DIR}/optimization/iSWIFT/cpp_wrapper
)
target_link_libraries(ctrl_lib PRIVATE dyn_lib)

# ===== High Level Library (basic test only) ===== #
# add_library(hl_lib
# 	"${LL_DIR}/src/forceUpdate.cpp"
# )

# ===== QP SWIFT Library ===== #
add_library(optim_lib
    "${LL_DIR}/optimization/iSWIFT/src/Auxilary.c"
    "${LL_DIR}/optimization/iSWIFT/src/Prime.c"
    "${LL_DIR}/optimization/iSWIFT/src/timer.c"
    "${LL_DIR}/optimization/iSWIFT/ldl/src/ldl.c"
    "${LL_DIR}/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp"
)
target_include_directories(optim_lib PUBLIC 
    ${EIGEN3_INCLUDE_DIRS}
    ${LL_DIR}/optimization/iSWIFT/include
    ${LL_DIR}/optimization/iSWIFT/ldl/include
    ${LL_DIR}/optimization/iSWIFT/cpp_wrapper
)


# ===== Executables ===== #
if (raisimOgre_FOUND)
    add_compile_definitions(RUNNING_SIM) # adds the toe offset to be used in simulation (exp needs no offset)
    add_executable(Run_Sim src/A1_Sim.cpp)
    target_link_libraries(Run_Sim PRIVATE ctrl_lib optim_lib dyn_lib raisim::raisimOgre ifopt::ifopt_ipopt ifopt::ifopt_snopt)
    target_compile_definitions(Run_Sim PRIVATE "-DEXAMPLE_ROBOT_RESOURCE_DIR=${LL_DIR}/rsc/")
# else()
    # add_executable(Basit_Experiment driver_scripts/qp_only_exp.cpp)
    # #target_link_libraries(Basit_Experiment PRIVATE ctrl_lib optim_lib dyn_lib ${EXTRA_LIBS} ${LCM} ${Boost_LIBRARIES} ifopt::ifopt_ipopt ifopt::ifopt_snopt)
    # target_link_libraries(Basit_Experiment PRIVATE ctrl_lib optim_lib dyn_lib ${EXTRA_LIBS} ${LCM} ${Boost_LIBRARIES} ifopt::ifopt_snopt)
    # target_include_directories(Basit_Experiment PRIVATE ${UT_INC})
    
    # add_executable(A1_Experiment_w_HL src/A1_Exp_w_HL.cpp)
    # target_link_libraries(A1_Experiment_w_HL PRIVATE ctrl_lib hl_lib optim_lib dyn_lib ${EXTRA_LIBS} ${LCM} ${Boost_LIBRARIES})
    # target_include_directories(A1_Experiment_w_HL PRIVATE ${UT_INC})
    
    # add_executable(restore src/restoreConfig.cpp)
    # target_link_libraries(restore PRIVATE ${EXTRA_LIBS} ${LCM} ${Boost_LIBRARIES})
    # target_include_directories(restore PRIVATE ${UT_INC})
endif()

# EOF