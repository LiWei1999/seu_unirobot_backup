add_library(vision
        vision.cpp)

target_link_libraries(vision
        opencv_core
        opencv_imgcodecs
        opencv_imgproc
        imageproc
        darknet
        server
        SL
        pthread
        rt
        stdc++
        cudart)
