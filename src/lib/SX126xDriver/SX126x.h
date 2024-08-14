#pragma once

#include "targets.h"
#include "SX126x_Regs.h"
#include "SX126x_hal.h"
#include "SX12xxDriverCommon.h"

#ifdef PLATFORM_ESP8266
#include <cstdint>
#endif

#define RADIO_SNR_SCALE 4

class SX126xDriver: public SX12xxDriverCommon
{
public:
    static SX126xDriver *instance;

    ///////////Radio Variables////////
    uint32_t timeout;

    ///////////////////////////////////

    ////////////////Configuration Functions/////////////
    SX126xDriver();
    bool Begin(uint32_t minimumFrequency, uint32_t maximumFrequency);
    void End();
    void SetTxIdleMode() { SetMode(SX126X_MODE_FS, SX12XX_Radio_All); }; // set Idle mode used when switching from RX to TX
    void Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq,
                uint8_t PreambleLength, bool InvertIQ, uint8_t PayloadLength, uint32_t interval, bool setFSKModulation,
                uint8_t fskSyncWord1, uint8_t fskSyncWord2, SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_All);
    void SetFrequencyHz(uint32_t freq, SX12XX_Radio_Number_t radioNumber);
    void SetFrequencyReg(uint32_t freq, SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_All);
    void SetRxTimeoutUs(uint32_t interval);
    void SetOutputPower(int8_t power, bool isSubGHz = true);
    void startCWTest(uint32_t freq, SX12XX_Radio_Number_t radioNumber);


    bool GetFrequencyErrorbool();
    // bool FrequencyErrorAvailable() const { return modeSupportsFei && (LastPacketSNRRaw > 0); }
    bool FrequencyErrorAvailable() const { return false; }

    void TXnb(uint8_t * data, uint8_t size, SX12XX_Radio_Number_t radioNumber);
    void RXnb(SX126X_RadioOperatingModes_t rxMode = SX126X_MODE_RX);

    uint32_t GetIrqStatus(SX12XX_Radio_Number_t radioNumber);
    void ClearIrqStatus(SX12XX_Radio_Number_t radioNumber);

    int8_t GetRssiInst(SX12XX_Radio_Number_t radioNumber);
    void GetLastPacketStats();

private:
    // constant used for no power change pending
    // must not be a valid power register value
    static const uint8_t PWRPENDING_NONE = 0x7f;

    // SX126X_RadioOperatingModes_t currOpmode;
    bool useFSK;
    bool modeSupportsFei;
    uint8_t pwrCurrentLF;
    uint8_t pwrPendingLF;
    uint8_t pwrCurrentHF; // HF = High Frequency
    uint8_t pwrPendingHF;
    bool pwrForceUpdate;
    bool radio1isSubGHz;
    bool radio2isSubGHz;
    SX126X_RadioOperatingModes_t fallBackMode;
    bool useFEC;

    void SetMode(SX126X_RadioOperatingModes_t OPmode, SX12XX_Radio_Number_t radioNumber);

    // LoRa functions
    void ConfigModParamsLoRa(uint8_t bw, uint8_t sf, uint8_t cr, SX12XX_Radio_Number_t radioNumber);
    void SetPacketParamsLoRa(uint8_t PreambleLength, SX126X_RadioLoRaPacketLengthsModes_t HeaderType,
                             uint8_t PayloadLength, uint8_t InvertIQ, SX12XX_Radio_Number_t radioNumber);
    // FSK functions
    void ConfigModParamsFSK(uint32_t Bitrate, uint8_t BWF, uint32_t Fdev, SX12XX_Radio_Number_t radioNumber);
    void SetPacketParamsFSK(uint8_t PreambleLength, uint8_t PayloadLength, SX12XX_Radio_Number_t radioNumber);
    void SetFSKSyncWord(uint8_t fskSyncWord1, uint8_t fskSyncWord2, SX12XX_Radio_Number_t radioNumber);

    void SetDioIrqParams();
    void SetDioAsRfSwitch();
    void CorrectRegisterForSF6(uint8_t sf, SX12XX_Radio_Number_t radioNumber);

    static void IsrCallback_1();
    static void IsrCallback_2();
    static void IsrCallback(SX12XX_Radio_Number_t radioNumber);
    bool RXnbISR(SX12XX_Radio_Number_t radioNumber); // ISR for non-blocking RX routine
    void TXnbISR(); // ISR for non-blocking TX routine
    void CommitOutputPower();
    void WriteOutputPower(uint8_t pwr, bool isSubGHz, SX12XX_Radio_Number_t radioNumber);
};
