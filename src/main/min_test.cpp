//This program acts like skybass.

#include <Arduino.h>
#include "min.h"
#include "min.c"
#include "min-irec.h"
#include "min_support.h"
//#define DEBUG_MIN

skyb_data_t skyb_data;
skyb_cmd_t skyb_cmd;

struct min_context min_ctx_sradio;
struct min_context min_ctx_esp;

void process_commands();

void min_application_handler(uint8_t min_id, uint8_t *min_payload, uint8_t len_payload, uint8_t port)
{
    switch (min_id)
    {
    case SKYB_CMD:
        //Serial.println("Got SKYB_CMD");
        if (len_payload == sizeof(skyb_cmd_t))
        {
            memcpy(&skyb_cmd, min_payload, len_payload);
            process_commands();
        }
        else
        {
            Serial.printf("Size Err: %i/%i", len_payload, sizeof(skyb_cmd_t));
        }
        break;
    case ESP_STATUS:
        //Serial.println("Got ESP_STATUS");
        min_send_frame(&min_ctx_sradio, min_id, min_payload, len_payload); //forward packet
        break;
    case ESP_ARM:
        //Serial.println("Got ESP_ARM");
        min_send_frame(&min_ctx_esp, min_id, min_payload, len_payload); //forward packet
        break;

    case ESP_STAGE:
        //Serial.println("Got ESP_STAGE");
        min_send_frame(&min_ctx_esp, min_id, min_payload, len_payload); //forward packet
        break;
    }
}

int main()
{
    Serial.begin(115200);  //USB Debug
    Serial2.begin(115200); //Serial to ESP
    Serial3.begin(115200); //Serial to Sradio
    skyb_data.latitude = 420.69;
    uint32_t timer = millis();

    min_init_context(&min_ctx_esp, 1);    //Context 1, Serial2
    min_init_context(&min_ctx_sradio, 2); //Context2, Serial3

    while (true)
    {
        //MIN Serial polling code
        char buf[32];
        size_t buf_len;
        if (Serial3.available() > 0)
        {
            buf_len = Serial3.readBytes(buf, 32U);
#ifdef DEBUG_MIN
            for (int i = 0; i < 32; i++)
            {
                Serial.printf("Sent: 0x%04x\n", buf[i]);
            }
            Serial.println();
#endif
        }
        else
        {
            buf_len = 0;
        }
        min_poll(&min_ctx_sradio, (uint8_t *)buf, (uint8_t)buf_len);

        if (Serial2.available() > 0)
        {
            buf_len = Serial2.readBytes(buf, 32U);
        }
        else
        {
            buf_len = 0;
        }
        min_poll(&min_ctx_esp, (uint8_t *)buf, (uint8_t)buf_len);

        if (millis() - timer > 2000)
        {
            timer = millis();
            min_send_frame(&min_ctx_sradio, SKYB_DATA, (uint8_t *)&skyb_data, sizeof(skyb_data));
        }
    }
}

void process_commands()
{
    if (skyb_cmd.reset)
    {
        Serial.println("reset");
    }
    if (skyb_cmd.ematch1)
    {
        Serial.println("ematch1");
    }
    if (skyb_cmd.ematch2)
    {
        Serial.println("ematch2");
    }
    if (skyb_cmd.ematch3)
    {
        Serial.println("ematch3");
    }
    if (skyb_cmd.ematch4)
    {
        Serial.println("ematch4");
    }
}

void min_tx_byte(uint8_t port, uint8_t byte)
{
    switch (port)
    {
    case 1:
        Serial2.write(&byte, 1U);
        break;
    case 2:
        Serial3.write(&byte, 1U);
        break;
    }
}

uint16_t min_tx_space(uint8_t port)
{
    switch (port)
    {
    case 1:
        return Serial2.availableForWrite();
        break;
    case 2:
        return Serial3.availableForWrite();
        break;
    }
    return 0;
}