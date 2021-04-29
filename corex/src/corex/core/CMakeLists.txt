cmake_minimum_required(VERSION 3.13)

add_library(corex-core SHARED
    Timer.cpp
    Scene.cpp
    SceneManager.cpp
    Application.cpp
    AssetManager.cpp
    DebugUI.cpp
    Camera.cpp
    CoreXNull.cpp
    Settings.cpp
    WindowManager.cpp
    asset_functions.cpp
    math_functions.cpp
    components/Text.cpp
    ds/Tree.hpp
    ds/TreeNode.hpp
    ds/Vec2.cpp
    ds/VecN.cpp
    systems/BaseSystem.cpp
    systems/KeyboardHandler.cpp
    systems/MouseHandler.cpp
    systems/SpritesheetAnimation.cpp
    systems/SysEventDispatcher.cpp
    main.cpp
    sdl_deleters.cpp
    utils.cpp
    allocator.cpp
)
target_link_libraries(corex-core
    ${SDL2_LIBRARIES}
)
