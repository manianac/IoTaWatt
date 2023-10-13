#include "IotaWatt.h"

uint32_t displayService(struct serviceBlock* _serviceBlock) {
    char szBuffer[64] = { 0 };
    trace(T_display, 0);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_unifont_tr);
    snprintf_P(szBuffer, sizeof(szBuffer), PSTR("%.f Samp/Cyc"), samplesPerCycle);
    u8g2.drawStr(8, 12, szBuffer);
    snprintf_P(szBuffer, sizeof(szBuffer), PSTR("%.1f Cyc/S"), cycleSampleRate);
    u8g2.drawStr(8, 25, szBuffer);
    snprintf_P(szBuffer, sizeof(szBuffer), PSTR("%0.1fV %.1fHz"), statRecord.accum1[0], frequency);
    u8g2.drawStr(8, 38, szBuffer);
    snprintf_P(szBuffer, sizeof(szBuffer), PSTR("RSSI %d"), WiFi.RSSI());
    u8g2.drawStr(8, 51, szBuffer);
    u8g2.sendBuffer();

    trace(T_display, 1);
    return displayServiceInterval;
}