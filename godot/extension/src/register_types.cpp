#include "register_types.h"
#include "maze_viewer.h"

#include <godot_cpp/godot.hpp>

using namespace godot;

void initialize_artificial_vision_module(ModuleInitializationLevel level) {
    if (level == MODULE_INITIALIZATION_LEVEL_SCENE) ClassDB::register_class<MazeViewer>();
}

void uninitialize_artificial_vision_module(ModuleInitializationLevel level) {}

extern "C" {
GDExtensionBool GDE_EXPORT artificial_vision_library_init(
        GDExtensionInterfaceGetProcAddress get_proc_address,
        const GDExtensionClassLibraryPtr library,
        GDExtensionInitialization *initialization) {
    GDExtensionBinding::InitObject init_object(get_proc_address, library, initialization);
    init_object.register_initializer(initialize_artificial_vision_module);
    init_object.register_terminator(uninitialize_artificial_vision_module);
    init_object.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
    return init_object.init();
}
}
