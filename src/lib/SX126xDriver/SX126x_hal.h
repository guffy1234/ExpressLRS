#pragma once

#include "SX126x_Regs.h"
#include "SX126x.h"

class SX126xHal
{
public:
    static SX126xHal *instance;

    SX126xHal();

    void init();
    void end();
    void reset(bool bootloader = false);

    void ICACHE_RAM_ATTR WriteCommand(uint16_t opcode, SX12XX_Radio_Number_t radioNumber);
    void ICACHE_RAM_ATTR WriteCommand(uint16_t opcode, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber);

    void ICACHE_RAM_ATTR ReadCommand(uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber);

    bool ICACHE_RAM_ATTR WaitOnBusy(SX12XX_Radio_Number_t radioNumber);

    static ICACHE_RAM_ATTR void dioISR_1();
    static ICACHE_RAM_ATTR void dioISR_2();
    void (*IsrCallback_1)();
    void (*IsrCallback_2)();

private:

};
