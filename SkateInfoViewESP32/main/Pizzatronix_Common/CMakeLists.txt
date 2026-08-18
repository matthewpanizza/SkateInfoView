idf_component_register(SRCS
    CANBus/CANAnalyzerStack.cpp
    Menu/MenuItem.cpp
    Menu/MenuSystem.cpp
    Power/PowerManager.cpp
    Power/PowerRegulator.cpp
    Serial/ISerialInterface.h
    Serial/ISerialInterface.cpp
    Serial/ISerialInterfaceCombiner.cpp
    INCLUDE_DIRS "." "include" "Serial" "Menu")
