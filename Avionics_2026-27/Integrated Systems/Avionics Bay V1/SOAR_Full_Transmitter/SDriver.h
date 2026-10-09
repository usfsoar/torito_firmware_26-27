#include <Adafruit_MCP23X17.h>

class SDriver {
    public:
        SDriver();
        void begin();
        void DriverActivate(int num);
        void DriverDeactivate(int num);

    private:
        Adafruit_MCP23X17 mcp;
        #define N0 0
        #define N1 1
        #define N2 2
        #define N3 3
        #define N4 4
        #define N5 5
        #define N6 6
        #define N7 7
};