#ifdef PJ_MD0
#include "7106_class.h"
namespace exIO
{
    Adafruit_MCP23X17 mcp;
    bool init(TwoWire &wire)
    {
        bool success = mcp.begin_I2C(0x20, &wire);
        if (success)
        {
            for (int i = 0; i < 16; i++)
            {
                mcp.pinMode(i, OUTPUT);
            }
        }
        return success;
    }
    void digitalWrite(uint8_t pin, uint8_t val)
    {
        pin < 64 ? arduino::digitalWrite(pin, val) : mcp.digitalWrite(pin - 64, val);
    }
    void pinMode(uint8_t pin, uint8_t val)
    {
        pin < 64 ? arduino::pinMode(pin, val) : mcp.pinMode(pin - 64, val);
    }
};
#endif