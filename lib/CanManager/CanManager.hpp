
// ignore -Wpedantic warnings for mcp2515.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <mcp2515.h>
#pragma GCC diagnostic pop

#include "Structs.h"

class CanManager {
public:

    CanManager(MCP2515& can_vcu, MCP2515& can_ssru);
    

    
private:
};