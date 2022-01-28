* uWave Packet mode polling

A project to demonstrate the transmission of telemetry information within the packet mode from several remote subscribers using one base station.

The "BaseStation" sketch is designed for the Arduino Mega board. This sketch:
- requests modem settings and changes them if necessary;
- Queries remote subscribers one by one in the specified address range. Moreover, each subscriber is requested twice: once by a standard request PT_ITG (https://docs.unavlab.com/documentation/EN/uWAVE/uWAVE_Protocol_Specification_en.html#218-ic_h2d_pt_itg) for measuring range and transmitting standard data: depth, water temperature, and supply voltage, and the second time it sends the address of the remote subscriber to the broadcast address. When the corresponding subscriber with this address receives this packet, it measures the voltage on one of the ADC pins and forms a 5-byte packet, where the first byte is some data identifier (in the example it is equal to zero), and the next four bytes contain a real 32-bit number - data with ADC.

Thus, it is demonstrated how to build a system based on uWave modems for collecting arbitrary telemetry data from remote subscribers.


More about uWave underwater acoustic modems: https://docs.unavlab.com/underwater_acoustic_modems_en.html#uwave
