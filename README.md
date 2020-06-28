# LoRa_Raspberry
Tested with Raspberry Pi 4B and LoRa with chip SX1276

1.Install wiringpi, update with version 2.52

cd /tmp
wget https://project-downloads.drogon.net/wiringpi-latest.deb

sudo dpkg -i wiringpi-latest.deb

Check version
gpio -v

2.Connect LoRa with Raspberry Pi 4B
<table>
	<tr>
		<td>LoRa SX1276</td><td>Raspberry Pi 4B</td>
	</tr>
	<tr>
		<td>3.3v</td><td>3.3v</td>
	</tr>
	<tr>
		<td>GND</td><td>GND</td>
	</tr>
	<tr>
		<td>NSS</td><td>GPIO.21</td>
	</tr>
	<tr>
		<td>DIO0</td><td>GPIO.22</td>
	</tr>
	<tr>
		<td>RESET</td><td>GPIO.23</td>
	</tr>
	<tr>
		<td>MOSI</td><td>MOSI</td>
	</tr>
	<tr>
		<td>MISO</td><td>MISO</td>
	</tr>
	<tr>
		<td>SCK</td><td>SCK</td>
	</tr>
</table>

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
