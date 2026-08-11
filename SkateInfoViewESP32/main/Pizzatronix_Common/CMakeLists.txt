idf_component_register(SRCS
    CANBus/CANAnalyzerStack.cpp
    Power/PowerManager.cpp
    Power/PowerRegulator.cpp
    Serial/ISerialInterface.h
    Serial/ISerialInterface.cpp
    Serial/ISerialInterfaceCombiner.cpp
    INCLUDE_DIRS "." "include" "Serial")
