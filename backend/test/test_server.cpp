/**
 * @file test_server.cpp
 * @brief OPC UA Test Server - Simulates the real PLC for development/testing
 *
 * This server creates an OPC UA address space identical to the real PLC,
 * allowing the entire ROS 2 system (backend, gui_app, demonstrator_tree)
 * to run without physical hardware.
 *
 * Usage:
 *   ros2 run backend test_server
 *   # Then in another terminal:
 *   ros2 run backend opc_bridge   (with opcua_test.yaml endpoint = localhost:4840)
 */

#include <open62541/plugin/log_stdout.h>
#include <open62541/server.h>
#include <open62541/server_config_default.h>

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <iomanip>

static volatile UA_Boolean running = true;

static void stopHandler(int sig) {
    (void)sig;
    UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "received ctrl-c");
    running = false;
}

/* ------------------------------------------------------------------ */
/*  ANSI color codes for pretty console output                        */
/* ------------------------------------------------------------------ */
#define CLR_RESET   "\033[0m"
#define CLR_YELLOW  "\033[1;33m"
#define CLR_GREEN   "\033[1;32m"
#define CLR_CYAN    "\033[1;36m"
#define CLR_MAGENTA "\033[1;35m"
#define CLR_BLUE    "\033[1;34m"
#define CLR_WHITE   "\033[1;37m"
#define CLR_DIM     "\033[2m"

/* ------------------------------------------------------------------ */
/*  Helper: get a short display name from the full OPC UA node id     */
/*  e.g. "ROS2_COMM"."MOD"."COBOT"  →  MOD / COBOT                   */
/* ------------------------------------------------------------------ */
static std::string prettyNodeName(const UA_NodeId *nodeId) {
    if(nodeId->identifierType != UA_NODEIDTYPE_STRING)
        return "<numeric>";
    std::string full((char*)nodeId->identifier.string.data,
                     nodeId->identifier.string.length);

    /* Strip leading "ROS2_COMM". to keep it shorter */
    const std::string prefix = "\"ROS2_COMM\".";
    if(full.rfind(prefix, 0) == 0)
        full = full.substr(prefix.size());

    /* Replace "." with " / " and strip quotes for readability */
    std::string nice;
    for(size_t i = 0; i < full.size(); i++) {
        if(full[i] == '"') continue;
        if(full[i] == '.' && i+1 < full.size() && full[i+1] == '"') {
            nice += " / ";
            continue;
        }
        nice += full[i];
    }
    return nice;
}

static std::string timestamp() {
    auto now = std::time(nullptr);
    auto *tm = std::localtime(&now);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%H:%M:%S", tm);
    return std::string(buf);
}

/* ------------------------------------------------------------------ */
/*  onWrite callbacks — print value changes to the console            */
/* ------------------------------------------------------------------ */
static void onWriteBool(UA_Server *, const UA_NodeId *,
                        void *, const UA_NodeId *nodeId,
                        void *, const UA_NumericRange *,
                        const UA_DataValue *data) {
    if(!data || !data->hasValue ||
       data->value.type != &UA_TYPES[UA_TYPES_BOOLEAN]) return;
    UA_Boolean v = *(UA_Boolean*)data->value.data;
    std::cout << CLR_DIM << timestamp() << CLR_RESET
              << CLR_YELLOW " [WRITE] " CLR_RESET
              << CLR_CYAN << prettyNodeName(nodeId) << CLR_RESET
              << " = " << (v ? CLR_GREEN "true" : CLR_MAGENTA "false")
              << CLR_RESET << std::endl;
}

static void onWriteInt16(UA_Server *, const UA_NodeId *,
                         void *, const UA_NodeId *nodeId,
                         void *, const UA_NumericRange *,
                         const UA_DataValue *data) {
    if(!data || !data->hasValue ||
       data->value.type != &UA_TYPES[UA_TYPES_INT16]) return;
    UA_Int16 v = *(UA_Int16*)data->value.data;
    std::cout << CLR_DIM << timestamp() << CLR_RESET
              << CLR_YELLOW " [WRITE] " CLR_RESET
              << CLR_CYAN << prettyNodeName(nodeId) << CLR_RESET
              << " = " << CLR_WHITE << v << CLR_RESET << std::endl;
}

static void onWriteDouble(UA_Server *, const UA_NodeId *,
                          void *, const UA_NodeId *nodeId,
                          void *, const UA_NumericRange *,
                          const UA_DataValue *data) {
    if(!data || !data->hasValue ||
       data->value.type != &UA_TYPES[UA_TYPES_DOUBLE]) return;
    UA_Double v = *(UA_Double*)data->value.data;
    std::cout << CLR_DIM << timestamp() << CLR_RESET
              << CLR_YELLOW " [WRITE] " CLR_RESET
              << CLR_CYAN << prettyNodeName(nodeId) << CLR_RESET
              << " = " << CLR_WHITE << std::fixed << std::setprecision(3)
              << v << CLR_RESET << std::endl;
}

/* ------------------------------------------------------------------ */
/*  Helper: register an onWrite callback on an existing node          */
/* ------------------------------------------------------------------ */
static void setWriteCb(UA_Server *server, UA_UInt16 ns,
                       const std::string &nodeId,
                       void (*cb)(UA_Server*, const UA_NodeId*,
                                  void*, const UA_NodeId*,
                                  void*, const UA_NumericRange*,
                                  const UA_DataValue*)) {
    UA_ValueCallback vcb;
    memset(&vcb, 0, sizeof(vcb));
    vcb.onWrite = cb;
    UA_Server_setVariableNode_valueCallback(server,
        UA_NODEID_STRING(ns, (char*)nodeId.c_str()), vcb);
}

/* ------------------------------------------------------------------ */
/*  Helper: add a Boolean variable under a parent + write callback    */
/* ------------------------------------------------------------------ */
static UA_StatusCode addBoolVar(UA_Server *server, UA_UInt16 ns,
                                const std::string &nodeId,
                                const std::string &parentId,
                                const char *browseName,
                                bool initVal = false) {
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    UA_Boolean val = initVal;
    UA_Variant_setScalar(&attr.value, &val, &UA_TYPES[UA_TYPES_BOOLEAN]);
    attr.displayName = UA_LOCALIZEDTEXT((char*)"en-US", (char*)browseName);
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_StatusCode rc = UA_Server_addVariableNode(server,
        UA_NODEID_STRING(ns, (char*)nodeId.c_str()),
        UA_NODEID_STRING(ns, (char*)parentId.c_str()),
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
        UA_QUALIFIEDNAME(ns, (char*)browseName),
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
        attr, NULL, NULL);
    if(rc == UA_STATUSCODE_GOOD) setWriteCb(server, ns, nodeId, onWriteBool);
    return rc;
}

/* ------------------------------------------------------------------ */
/*  Helper: add an Int16 variable under a parent + write callback     */
/* ------------------------------------------------------------------ */
static UA_StatusCode addInt16Var(UA_Server *server, UA_UInt16 ns,
                                 const std::string &nodeId,
                                 const std::string &parentId,
                                 const char *browseName,
                                 int16_t initVal = 0) {
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    UA_Int16 val = initVal;
    UA_Variant_setScalar(&attr.value, &val, &UA_TYPES[UA_TYPES_INT16]);
    attr.displayName = UA_LOCALIZEDTEXT((char*)"en-US", (char*)browseName);
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_StatusCode rc = UA_Server_addVariableNode(server,
        UA_NODEID_STRING(ns, (char*)nodeId.c_str()),
        UA_NODEID_STRING(ns, (char*)parentId.c_str()),
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
        UA_QUALIFIEDNAME(ns, (char*)browseName),
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
        attr, NULL, NULL);
    if(rc == UA_STATUSCODE_GOOD) setWriteCb(server, ns, nodeId, onWriteInt16);
    return rc;
}

/* ------------------------------------------------------------------ */
/*  Helper: add a Double variable under a parent + write callback     */
/* ------------------------------------------------------------------ */
static UA_StatusCode addDoubleVar(UA_Server *server, UA_UInt16 ns,
                                  const std::string &nodeId,
                                  const std::string &parentId,
                                  const char *browseName,
                                  double initVal = 0.0) {
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    UA_Double val = initVal;
    UA_Variant_setScalar(&attr.value, &val, &UA_TYPES[UA_TYPES_DOUBLE]);
    attr.displayName = UA_LOCALIZEDTEXT((char*)"en-US", (char*)browseName);
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_StatusCode rc = UA_Server_addVariableNode(server,
        UA_NODEID_STRING(ns, (char*)nodeId.c_str()),
        UA_NODEID_STRING(ns, (char*)parentId.c_str()),
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
        UA_QUALIFIEDNAME(ns, (char*)browseName),
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
        attr, NULL, NULL);
    if(rc == UA_STATUSCODE_GOOD) setWriteCb(server, ns, nodeId, onWriteDouble);
    return rc;
}

/* ------------------------------------------------------------------ */
/*  Helper: add an Object node under a parent                         */
/* ------------------------------------------------------------------ */
static UA_StatusCode addObject(UA_Server *server, UA_UInt16 ns,
                               const std::string &nodeId,
                               const std::string &parentId,
                               const char *browseName) {
    UA_ObjectAttributes attr = UA_ObjectAttributes_default;
    attr.displayName = UA_LOCALIZEDTEXT((char*)"en-US", (char*)browseName);
    return UA_Server_addObjectNode(server,
        UA_NODEID_STRING(ns, (char*)nodeId.c_str()),
        UA_NODEID_STRING(ns, (char*)parentId.c_str()),
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
        UA_QUALIFIEDNAME(ns, (char*)browseName),
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE),
        attr, NULL, NULL);
}

/* ================================================================== */
/*  Build the full ROS2_COMM OPC UA address space                     */
/* ================================================================== */
static UA_StatusCode addROS2CommNodes(UA_Server *server, UA_UInt16 ns) {
    UA_StatusCode retval;

    /* ---- Root: ROS2_COMM ---- */
    UA_ObjectAttributes ros2commAttr = UA_ObjectAttributes_default;
    ros2commAttr.displayName = UA_LOCALIZEDTEXT((char*)"en-US", (char*)"ROS2_COMM");
    retval = UA_Server_addObjectNode(server,
        UA_NODEID_STRING(ns, (char*)"ROS2_COMM"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
        UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
        UA_QUALIFIEDNAME(ns, (char*)"ROS2_COMM"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE),
        ros2commAttr, NULL, NULL);
    if(retval != UA_STATUSCODE_GOOD) {
        std::cerr << "Failed to add ROS2_COMM object" << std::endl;
        return retval;
    }

    const std::string ROOT = "ROS2_COMM";

    /* ---- Top-level Int16 variables ---- */
    addInt16Var(server, ns, "\"ROS2_COMM\".\"STATUS\"",   ROOT, "STATUS",  0);
    addInt16Var(server, ns, "\"ROS2_COMM\".\"MODE\"",     ROOT, "MODE",    0);
    addInt16Var(server, ns, "\"ROS2_COMM\".\"COMMAND\"",  ROOT, "COMMAND", 0);
    addInt16Var(server, ns, "\"ROS2_COMM\".\"SPEED\"",    ROOT, "SPEED",   1000);

    /* ---- Top-level Double variables (slider go positions) ---- */
    addDoubleVar(server, ns, "\"ROS2_COMM\".\"GO_TO_POS_1\"", ROOT, "GO_TO_POS_1", 0.0);
    addDoubleVar(server, ns, "\"ROS2_COMM\".\"GO_TO_POS_2\"", ROOT, "GO_TO_POS_2", 0.0);

    /* ============================================================ */
    /*  MOD object  (mode bits)                                     */
    /* ============================================================ */
    const std::string MOD = "\"ROS2_COMM\".\"MOD\"";
    addObject(server, ns, MOD, ROOT, "MOD");

    const char* modFields[] = {
        "STARTUP", "CALIBRATION", "LEARNING", "MAINTENANCE",
        "EMERGENCY", "COBOT", "FULLY_AUTOMATIC", "SHUTDOWN_MODE"
    };
    for(size_t i = 0; i < 8; i++) {
        std::string nid = MOD + ".\"" + modFields[i] + "\"";
        addBoolVar(server, ns, nid, MOD, modFields[i]);
    }

    /* ---- MOD > Workcell_Status (slider actual positions) ---- */
    const std::string WC = MOD + ".\"Workcell_Status\"";
    addObject(server, ns, WC, MOD, "Workcell_Status");
    addDoubleVar(server, ns, WC + ".\"Slider_1_actual position-linear\"", WC,
                 "Slider_1_actual position-linear", 0.0);
    addDoubleVar(server, ns, WC + ".\"Slider_2_actual position-linear\"", WC,
                 "Slider_2_actual position-linear", 0.0);

    /* ============================================================ */
    /*  STAT object  (status bits)                                  */
    /* ============================================================ */
    const std::string STAT = "\"ROS2_COMM\".\"STAT\"";
    addObject(server, ns, STAT, ROOT, "STAT");

    const char* statFields[] = {
        "STARTUP", "CALIBRATION", "LEARNING",
        "MAINTENANCE", "EMERGENCY", "COBOT"
    };
    for(size_t i = 0; i < 6; i++) {
        std::string nid = STAT + ".\"" + statFields[i] + "\"";
        addBoolVar(server, ns, nid, STAT, statFields[i]);
    }

    /* ---- STAT > Robot_Sensing_Status ---- */
    const std::string SENS = STAT + ".\"Robot_Sensing_Status\"";
    addObject(server, ns, SENS, STAT, "Robot_Sensing_Status");

    const char* sensingFields[] = {
        "robothome_safetransfer", "sensing-finised", "touchsensing-finished",
        "sensing-active", "touchsensing-active", "slide command", "running"
    };
    for(auto &f : sensingFields) {
        std::string nid = SENS + ".\"" + f + "\"";
        addBoolVar(server, ns, nid, SENS, f);
    }

    /* ---- STAT > Robot_Cleaning_Status ---- */
    const std::string CLEAN = STAT + ".\"Robot_Cleaning_Status\"";
    addObject(server, ns, CLEAN, STAT, "Robot_Cleaning_Status");

    const char* cleaningFields[] = {
        "robothome_safetransfer", "cleaning-finished",
        "cleaning-active", "slide command", "running"
    };
    for(auto &f : cleaningFields) {
        std::string nid = CLEAN + ".\"" + f + "\"";
        addBoolVar(server, ns, nid, CLEAN, f);
    }

    return UA_STATUSCODE_GOOD;
}

/* ================================================================== */
/*  main                                                              */
/* ================================================================== */
int main(void) {
    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    UA_Server *server = UA_Server_new();
    UA_ServerConfig *config = UA_Server_getConfig(server);
    UA_ServerConfig_setMinimal(config, 4840, NULL);

    /* Namespace 3 icin 2 tane eklememiz gerekiyor (0,1 default) */
    UA_UInt16 ns2 = UA_Server_addNamespace(server, "urn:open62541:dummy");
    UA_UInt16 nsIdx = UA_Server_addNamespace(server, "urn:open62541:ros2comm");

    std::cout << "Namespace indices created: " << ns2 << ", " << nsIdx << std::endl;

    if(nsIdx != 3) {
        std::cerr << "Warning: Expected namespace 3, got " << nsIdx << std::endl;
        std::cerr << "Update YAML to use namespace_index: " << nsIdx << std::endl;
    }

    UA_StatusCode retval = addROS2CommNodes(server, nsIdx);
    if(retval != UA_STATUSCODE_GOOD) {
        UA_Server_delete(server);
        return EXIT_FAILURE;
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << " OPC UA Test Server" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << " Endpoint : opc.tcp://localhost:4840" << std::endl;
    std::cout << " Namespace: " << nsIdx << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << " Nodes:" << std::endl;
    std::cout << "   Int16 : STATUS, MODE, COMMAND, SPEED" << std::endl;
    std::cout << "   Double: GO_TO_POS_1, GO_TO_POS_2" << std::endl;
    std::cout << "   MOD   : 8 bool fields + Workcell_Status" << std::endl;
    std::cout << "   STAT  : 6 bool fields" << std::endl;
    std::cout << "   Sensing : 7 bool fields" << std::endl;
    std::cout << "   Cleaning: 5 bool fields" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << " Press Ctrl+C to stop" << std::endl;
    std::cout << "========================================\n" << std::endl;

    retval = UA_Server_run(server, &running);

    UA_Server_delete(server);
    return retval == UA_STATUSCODE_GOOD ? EXIT_SUCCESS : EXIT_FAILURE;
}