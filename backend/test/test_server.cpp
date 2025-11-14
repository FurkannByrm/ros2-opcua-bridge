#include <open62541/plugin/log_stdout.h>
#include <open62541/server.h>
#include <open62541/server_config_default.h>

#include <signal.h>
#include <stdlib.h>
#include <iostream>

static volatile UA_Boolean running = true;

static void stopHandler(int sig) {
    UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "received ctrl-c");
    running = false;
}

static UA_StatusCode addROS2CommNodes(UA_Server *server, UA_UInt16 nsIdx) {
    UA_StatusCode retval;
    
    // ROS2_COMM parent object
    UA_ObjectAttributes ros2commAttr = UA_ObjectAttributes_default;
    ros2commAttr.displayName = UA_LOCALIZEDTEXT((char*)"en-US", (char*)"ROS2_COMM");
    retval = UA_Server_addObjectNode(server,
        UA_NODEID_STRING(nsIdx, (char*)"ROS2_COMM"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
        UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
        UA_QUALIFIEDNAME(nsIdx, (char*)"ROS2_COMM"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE),
        ros2commAttr, NULL, NULL);
    
    if(retval != UA_STATUSCODE_GOOD) {
        std::cerr << "Failed to add ROS2_COMM object" << std::endl;
        return retval;
    }
    
    // STATUS node (Int16)
    UA_VariableAttributes statusAttr = UA_VariableAttributes_default;
    UA_Int16 statusValue = 0;
    UA_Variant_setScalar(&statusAttr.value, &statusValue, &UA_TYPES[UA_TYPES_INT16]);
    statusAttr.displayName = UA_LOCALIZEDTEXT((char*)"en-US", (char*)"STATUS");
    statusAttr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    retval = UA_Server_addVariableNode(server,
        UA_NODEID_STRING(nsIdx, (char*)"\"ROS2_COMM\".\"STATUS\""),
        UA_NODEID_STRING(nsIdx, (char*)"ROS2_COMM"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
        UA_QUALIFIEDNAME(nsIdx, (char*)"STATUS"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
        statusAttr, NULL, NULL);
    
    // MODE node (Int16)
    UA_VariableAttributes modeAttr = UA_VariableAttributes_default;
    UA_Int16 modeValue = 0;
    UA_Variant_setScalar(&modeAttr.value, &modeValue, &UA_TYPES[UA_TYPES_INT16]);
    modeAttr.displayName = UA_LOCALIZEDTEXT((char*)"en-US", (char*)"MODE");
    modeAttr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_Server_addVariableNode(server,
        UA_NODEID_STRING(nsIdx, (char*)"\"ROS2_COMM\".\"MODE\""),
        UA_NODEID_STRING(nsIdx, (char*)"ROS2_COMM"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
        UA_QUALIFIEDNAME(nsIdx, (char*)"MODE"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
        modeAttr, NULL, NULL);
    
    // COMMAND node (Int16)
    UA_VariableAttributes cmdAttr = UA_VariableAttributes_default;
    UA_Int16 cmdValue = 0;
    UA_Variant_setScalar(&cmdAttr.value, &cmdValue, &UA_TYPES[UA_TYPES_INT16]);
    cmdAttr.displayName = UA_LOCALIZEDTEXT((char*)"en-US", (char*)"COMMAND");
    cmdAttr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_Server_addVariableNode(server,
        UA_NODEID_STRING(nsIdx, (char*)"\"ROS2_COMM\".\"COMMAND\""),
        UA_NODEID_STRING(nsIdx, (char*)"ROS2_COMM"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
        UA_QUALIFIEDNAME(nsIdx, (char*)"COMMAND"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
        cmdAttr, NULL, NULL);
    
    // SPEED node (Int16)
    UA_VariableAttributes speedAttr = UA_VariableAttributes_default;
    UA_Int16 speedValue = 1000;
    UA_Variant_setScalar(&speedAttr.value, &speedValue, &UA_TYPES[UA_TYPES_INT16]);
    speedAttr.displayName = UA_LOCALIZEDTEXT((char*)"en-US", (char*)"SPEED");
    speedAttr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_Server_addVariableNode(server,
        UA_NODEID_STRING(nsIdx, (char*)"\"ROS2_COMM\".\"SPEED\""),
        UA_NODEID_STRING(nsIdx, (char*)"ROS2_COMM"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
        UA_QUALIFIEDNAME(nsIdx, (char*)"SPEED"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
        speedAttr, NULL, NULL);
    
    // MOD object
    UA_ObjectAttributes modAttr = UA_ObjectAttributes_default;
    modAttr.displayName = UA_LOCALIZEDTEXT((char*)"en-US", (char*)"MOD");
    UA_Server_addObjectNode(server,
        UA_NODEID_STRING(nsIdx, (char*)"\"ROS2_COMM\".\"MOD\""),
        UA_NODEID_STRING(nsIdx, (char*)"ROS2_COMM"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
        UA_QUALIFIEDNAME(nsIdx, (char*)"MOD"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE),
        modAttr, NULL, NULL);
    
    // MOD fields
    const char* modFields[] = {"STARTUP", "CALIBRATION", "LEARNING", 
                               "MAINTENANCE", "EMERGENCY", "COBOT", 
                               "FULLY_AUTOMATIC", "SHUTDOWN_MODE"};
    
    for(size_t i = 0; i < 8; i++) {
        UA_VariableAttributes attr = UA_VariableAttributes_default;
        UA_Boolean val = false;
        UA_Variant_setScalar(&attr.value, &val, &UA_TYPES[UA_TYPES_BOOLEAN]);
        attr.displayName = UA_LOCALIZEDTEXT((char*)"en-US", (char*)modFields[i]);
        attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
        
        std::string nodeId = "\"ROS2_COMM\".\"MOD\".\"" + std::string(modFields[i]) + "\"";
        UA_Server_addVariableNode(server,
            UA_NODEID_STRING(nsIdx, (char*)nodeId.c_str()),
            UA_NODEID_STRING(nsIdx, (char*)"\"ROS2_COMM\".\"MOD\""),
            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
            UA_QUALIFIEDNAME(nsIdx, (char*)modFields[i]),
            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
            attr, NULL, NULL);
    }
    
    // STAT object
    UA_ObjectAttributes statAttr = UA_ObjectAttributes_default;
    statAttr.displayName = UA_LOCALIZEDTEXT((char*)"en-US", (char*)"STAT");
    UA_Server_addObjectNode(server,
        UA_NODEID_STRING(nsIdx, (char*)"\"ROS2_COMM\".\"STAT\""),
        UA_NODEID_STRING(nsIdx, (char*)"ROS2_COMM"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
        UA_QUALIFIEDNAME(nsIdx, (char*)"STAT"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE),
        statAttr, NULL, NULL);
    
    // STAT fields
    const char* statFields[] = {"STARTUP", "CALIBRATION", "LEARNING", 
                                "MAINTENANCE", "EMERGENCY", "COBOT"};
    
    for(size_t i = 0; i < 6; i++) {
        UA_VariableAttributes attr = UA_VariableAttributes_default;
        UA_Boolean val = false;
        UA_Variant_setScalar(&attr.value, &val, &UA_TYPES[UA_TYPES_BOOLEAN]);
        attr.displayName = UA_LOCALIZEDTEXT((char*)"en-US", (char*)statFields[i]);
        attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
        
        std::string nodeId = "\"ROS2_COMM\".\"STAT\".\"" + std::string(statFields[i]) + "\"";
        UA_Server_addVariableNode(server,
            UA_NODEID_STRING(nsIdx, (char*)nodeId.c_str()),
            UA_NODEID_STRING(nsIdx, (char*)"\"ROS2_COMM\".\"STAT\""),
            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
            UA_QUALIFIEDNAME(nsIdx, (char*)statFields[i]),
            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
            attr, NULL, NULL);
    }
    
    return UA_STATUSCODE_GOOD;
}

int main(void) {
    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    UA_Server *server = UA_Server_new();
    UA_ServerConfig *config = UA_Server_getConfig(server);
    UA_ServerConfig_setDefault(config);
    
    // Namespace 3 için 2 tane eklememiz gerekiyor (0,1 default var)
    UA_String ns2Uri = UA_STRING((char*)"urn:open62541:dummy");
    UA_UInt16 ns2 = UA_Server_addNamespace(server, (const char*)ns2Uri.data);
    
    UA_String ns3Uri = UA_STRING((char*)"urn:open62541:ros2comm");
    UA_UInt16 nsIdx = UA_Server_addNamespace(server, (const char*)ns3Uri.data);
    
    std::cout << "Namespace indices created: " << ns2 << ", " << nsIdx << std::endl;
    
    if(nsIdx != 3) {
        std::cerr << "Warning: Expected namespace 3, got " << nsIdx << std::endl;
        std::cerr << "Update YAML to use namespace_index: " << nsIdx << std::endl;
    }

    // Nodes ekle
    UA_StatusCode retval = addROS2CommNodes(server, nsIdx);
    if(retval != UA_STATUSCODE_GOOD) {
        UA_Server_delete(server);
        return EXIT_FAILURE;
    }

    std::cout << "OPC UA Test Server started on opc.tcp://localhost:4840" << std::endl;
    std::cout << "Namespace " << nsIdx << " - ROS2_COMM structure" << std::endl;
    std::cout << "Node format: \"ROS2_COMM\".\"SPEED\" etc." << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;

    retval = UA_Server_run(server, &running);

    UA_Server_delete(server);
    return retval == UA_STATUSCODE_GOOD ? EXIT_SUCCESS : EXIT_FAILURE;
}