set(SOURCE_FILES
        main/cpp/ihmc-yovariables.hpp
        main/cpp/yo-registry-change-listener.hpp
        main/cpp/yo-namespace.hpp
        main/cpp/yo-namespace.cpp
        main/cpp/yo-tools.hpp
        main/cpp/yo-tools.cpp
        main/cpp/yo-registries.hpp
        main/cpp/yo-registries.cpp
        main/cpp/yo-variables.hpp
        main/cpp/yo-variables.cpp
        )

add_library(yo-variables SHARED ${SOURCE_FILES})
target_include_directories(yo-variables PUBLIC main/cpp)
target_link_libraries(yo-variables PUBLIC
        Eigen3::Eigen)

install(TARGETS yo-variables
        EXPORT targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)
install(DIRECTORY
        main/cpp/
        DESTINATION "${INSTALL_INC_DIR}"
        FILES_MATCHING PATTERN "*.hpp")