# LoRa_Raspberry
Tested with Raspberry Pi 4B and LoRa with chip SX1276

1.Install wiringpi, update with version 2.52

cd /tmp
wget https://project-downloads.drogon.net/wiringpi-latest.deb
sudo dpkg -i wiringpi-latest.deb

Check version
gpio -v

Connect LoRa with Raspberry Pi 4B
_________________________________________________
|	LoRa SX1276	|	Raspberry Pi 4B	|
_________________________________________________
|	3.3v		|	3.3v		|
_________________________________________________
|	GND		|	GND		|
_________________________________________________
|	NSS		|	GPIO.21		|
_________________________________________________
|	DIO0		|	GPIO.22		|
_________________________________________________
|	RESET		|	GPIO.23		|
_________________________________________________
|	MOSI		|	MOSI		|
_________________________________________________
|	MISO		|	MISO		|
_________________________________________________
|	SCK		|	SKC		|
_________________________________________________

3.Compile code with -lwiringpi options


My LoRa library have this public functions -

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
