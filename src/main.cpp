#include <Arduino.h>
#include <mcp2515.h>
#include <LiquidCrystal_I2C.h>

#include "Scheduler.hpp"

#include "pinMap.h"
#include "Structs.h"

#include "CanManager.hpp"
#include "Radio.hpp"
#include "Screen.hpp"

MCP2515 can_vcu(CAN0_CS);
MCP2515 can_ssru(CAN1_CS);

MCP2515 cans[NUM_MCP] = {can_vcu, can_ssru};

CanManager can_manager{can_vcu, can_ssru};
Radio radio{};
Screen screen{};

Scheduler<3, NUM_MCP + 2> scheduler(
    10000,  // period_us
    500,    // spin_threshold_us
    *micros // current_time_us function pointer
);

void setup()
{
    for (int i = 0; i < NUM_IN; ++i)
    {

        pinMode();
        digitalWrite(, LOW);
    }
    for (int i = 0; i < NUM_OUT; ++i)
    {

        pinMode();
    }

    for (int i = 0; i < NUM_MCP; ++i)
    {
        cans[i].reset();
        cans[i].setBitrate(CAN_500KBPS, MCP_20MHZ);
        cans[i].setNormalMode();
    }

    // some sort of synchronization function in can manager to get SSRU on same cycle as VCU, see Scheduler.synchronize() for reference
    can_manager.synchronize();

    scheduler.addTask(McpIndex::Vcu, schedulerVcu, 1);
    scheduler.addTask(McpIndex::Ssru, schedulerSsru, 1);
    scheduler.addTask(McpIndex::Screen, schedulerScreen, 1);
    scheduler.addTask(McpIndex::Radio, schedulerRadio, 1);
}

void loop()
{
    scheduler.update();
}