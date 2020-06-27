#ifndef LORA_RASPI_H
#define LORA_RASPI_H

#include <stdint.h>
#include <cstdio>
#include <cstdlib>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
typedef bool boolean;
typedef unsigned char byte;

using namespace std;

class LoRa {
private:
    int _ssPin;
    int _dio0;
    int _RST;
    int _SPIFreq;
    uint32_t _LoRaFreq;
    uint8_t _sw;
    uint8_t _sf;
    byte _receivedbytes;

public:
	void setPin(int ssPin, int dio0, int RST);
	void setSPIFrequency(uint32_t frequency);
	void selectreceiver();
	void unselectreceiver();
	byte readReg(byte addr);
	void writeReg(byte addr, byte value);
	void opmode (uint8_t mode);
	void opmodeLora();
	void setFrequency(uint32_t frequency);
	void setSyncWord(uint8_t sw);
	void setSpreadingFactor(uint8_t sf);
	boolean receive(byte *payload);
	int packetRssi();
	int Rssi();
	long int packetSnr();
	void receivepacket();
	void configPower(int8_t pw);
	void writeBuf(byte addr, byte *value, byte len);
	void txlora(byte *frame, byte datalen);
	void configTransmitter();
	void configReceiver();
	uint8_t getSF();
	uint32_t getLoRaFreq();
	int ReceivedBytes();
	boolean dio0State();
	void dumpRegisters();
	LoRa();
	~LoRa();
	int begin(uint32_t frequency);
};
#endif
