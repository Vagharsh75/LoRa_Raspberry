#include "LoRa_RasPi.h"

// #############################################
// #############################################

#define REG_FIFO                    0x00
#define REG_OPMODE                  0x01
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_RX_NB_BYTES             0x13
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_PKT_RSSI_VALUE          0x1A
#define REG_RSSI_VALUE              0x1B
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_SYMB_TIMEOUT_LSB  	    0x1F
#define REG_PKT_SNR_VALUE           0x19
#define REG_PAYLOAD_LENGTH          0x22
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_MAX_PAYLOAD_LENGTH 	    0x23
#define REG_HOP_PERIOD              0x24
#define REG_SYNC_WORD               0x39
#define REG_VERSION                 0x42
#define REG_DETECTION_OPTIMIZE	    0x31
#define REG_DETECTION_THRESHOLD     0x37


#define PAYLOAD_LENGTH              0x40

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23
#define LNA_OFF_GAIN                0x00
#define LNA_LOW_GAIN                0x20

#define RegDioMapping1              0x40 // common
#define RegDioMapping2              0x41 // common

#define RegPaConfig                 0x09 // common
#define RegPaRamp                   0x0A // common
#define RegPaDac                    0x5A // common

#define SX72_MC2_FSK                0x00
#define SX72_MC2_SF7                0x70
#define SX72_MC2_SF8                0x80
#define SX72_MC2_SF9                0x90
#define SX72_MC2_SF10               0xA0
#define SX72_MC2_SF11               0xB0
#define SX72_MC2_SF12               0xC0

#define SX72_MC1_LOW_DATA_RATE_OPTIMIZE  0x01 // mandated for SF11 and SF12

// sx1276 RegModemConfig1
#define SX1276_MC1_BW_125            0x70
#define SX1276_MC1_BW_250            0x80
#define SX1276_MC1_BW_500            0x90
#define SX1276_MC1_CR_4_5            0x02
#define SX1276_MC1_CR_4_6            0x04
#define SX1276_MC1_CR_4_7            0x06
#define SX1276_MC1_CR_4_8            0x08

#define SX1276_MC1_IMPLICIT_HEADER_MODE_ON    0x01

// sx1276 RegModemConfig2
#define SX1276_MC2_RX_PAYLOAD_CRCON        0x04

// sx1276 RegModemConfig3
#define SX1276_MC3_LOW_DATA_RATE_OPTIMIZE  0x08
#define SX1276_MC3_AGCAUTO                 0x04

// preamble for lora networks (nibbles swapped)
#define LORA_MAC_PREAMBLE                  0x34

#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1 0x0A
#ifdef LMIC_SX1276
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x70
#elif LMIC_SX1272
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x74
#endif

// FRF
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08

#define FRF_MSB                  0xD9 // 868.1 Mhz
#define FRF_MID                  0x06
#define FRF_LSB                  0x66

// ----------------------------------------
// Constants for radio registers
#define OPMODE_LORA      0x80
#define OPMODE_MASK      0x07
#define OPMODE_SLEEP     0x00
#define OPMODE_STANDBY   0x01
#define OPMODE_FSTX      0x02
#define OPMODE_TX        0x03
#define OPMODE_FSRX      0x04
#define OPMODE_RX        0x05
#define OPMODE_RX_SINGLE 0x06
#define OPMODE_CAD       0x07

// ----------------------------------------
// Bits masking the corresponding IRQs from the radio
#define IRQ_LORA_RXTOUT_MASK 0x80
#define IRQ_LORA_RXDONE_MASK 0x40
#define IRQ_LORA_CRCERR_MASK 0x20
#define IRQ_LORA_HEADER_MASK 0x10
#define IRQ_LORA_TXDONE_MASK 0x08
#define IRQ_LORA_CDDONE_MASK 0x04
#define IRQ_LORA_FHSSCH_MASK 0x02
#define IRQ_LORA_CDDETD_MASK 0x01

// DIO function mappings                D0D1D2D3
#define MAP_DIO0_LORA_RXDONE   0x00  // 00------
#define MAP_DIO0_LORA_TXDONE   0x40  // 01------
#define MAP_DIO1_LORA_RXTOUT   0x00  // --00----
#define MAP_DIO1_LORA_NOP      0x30  // --11----
#define MAP_DIO2_LORA_NOP      0xC0  // ----11--

// #############################################
// #############################################


#define LORA_DEFAULT_SS_PIN    21
#define LORA_DEFAULT_DIO0_PIN  22
#define LORA_DEFAULT_RESET_PIN 23
#define SPISPEED               500000

static const int CHANNEL = 0;
char message[256];
bool isBeginCalled = false;
bool sx1272 = true;
enum sf_t { SF6=6, SF7, SF8, SF9, SF10, SF11, SF12 };

void LoRa::setPin(int ssPin=LORA_DEFAULT_SS_PIN, int dio0=LORA_DEFAULT_DIO0_PIN, int RST=LORA_DEFAULT_RESET_PIN)
{
    if (!isBeginCalled) {
        this->_ssPin = ssPin;
        this->_dio0 = dio0;
        this->_RST = RST;
        wiringPiSetup ();
        pinMode(this->_ssPin, OUTPUT);
        pinMode(this->_dio0, INPUT);
        pinMode(this->_RST, OUTPUT);
    } else {
        printf("Call setPin() before begin()\n");
    }
}

void LoRa::setSPIFrequency(uint32_t frequency=SPISPEED)
{
    this->_SPIFreq = frequency;
    if (!isBeginCalled) {
        wiringPiSPISetup(CHANNEL, frequency);
    } else {
        printf("Call setSPIFrequency() before begin()\n");
    }
}

void LoRa::selectreceiver()
{
    digitalWrite(this->_ssPin, LOW);
}

void LoRa::unselectreceiver()
{
    digitalWrite(this->_ssPin, HIGH);
}

byte LoRa::readReg(byte addr)
{
    byte spibuf[2];

    selectreceiver();
    spibuf[0] = addr & 0x7F;
    spibuf[1] = 0x00;
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);
    unselectreceiver();

    return spibuf[1];
}

void LoRa::writeReg(byte addr, byte value)
{
    unsigned char spibuf[2];

    spibuf[0] = addr | 0x80;
    spibuf[1] = value;
    selectreceiver();
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);
    unselectreceiver();
}

void LoRa::opmode (uint8_t mode) {
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & ~OPMODE_MASK) | mode);
}

void LoRa::opmodeLora() {
    uint8_t u = OPMODE_LORA;
    if (sx1272 == false)
        u |= 0x8;   // TBD: sx1276 high freq
    writeReg(REG_OPMODE, u);
}

void LoRa::setFrequency(uint32_t frequency)
{
    this->_LoRaFreq = frequency;
    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
    writeReg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    writeReg(REG_FRF_MID, (uint8_t)(frf >> 8));
    writeReg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void LoRa::setSyncWord(uint8_t sw=0x34)
{
    this->_sw = sw;
    writeReg(REG_SYNC_WORD, sw);
}

void LoRa::setSpreadingFactor(uint8_t sf=7)
{
    this->_sf = sf;
    if (sf < 6) {
        sf = 6;
    } else if (sf > 12) {
        sf = 12;
    }

    if (sf == 6) {
        writeReg(REG_DETECTION_OPTIMIZE, 0xc5);
        writeReg(REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        writeReg(REG_DETECTION_OPTIMIZE, 0xc3);
        writeReg(REG_DETECTION_THRESHOLD, 0x0a);
    }

    if (sx1272) {
        if (sf == SF11 || sf == SF12) {
            writeReg(REG_MODEM_CONFIG,0x0B);
        } else {
            writeReg(REG_MODEM_CONFIG,0x0A);
        }
        writeReg(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
    } else {
        if (sf == SF11 || sf == SF12) {
            writeReg(REG_MODEM_CONFIG3,0x0C);
        } else {
            writeReg(REG_MODEM_CONFIG3,0x04);
        }
        writeReg(REG_MODEM_CONFIG,0x72);
        writeReg(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
    }

    if (sf == SF10 || sf == SF11 || sf == SF12) {
        writeReg(REG_SYMB_TIMEOUT_LSB,0x05);
    } else {
        writeReg(REG_SYMB_TIMEOUT_LSB,0x08);
    }
}

boolean LoRa::receive(char *payload) {
    // clear rxDone
    writeReg(REG_IRQ_FLAGS, 0x40);

    int irqflags = readReg(REG_IRQ_FLAGS);

    //  payload crc: 0x20
    if((irqflags & 0x20) == 0x20)
    {
        printf("CRC error\n");
        writeReg(REG_IRQ_FLAGS, 0x20);
        return false;
    } else {

        byte currentAddr = readReg(REG_FIFO_RX_CURRENT_ADDR);
        byte receivedCount = readReg(REG_RX_NB_BYTES);
        this->_receivedbytes = receivedCount;

        writeReg(REG_FIFO_ADDR_PTR, currentAddr);

        for(int i = 0; i < receivedCount; i++)
        {
            payload[i] = (char)readReg(REG_FIFO);
        }
    }
    return true;
}

int LoRa::packetRssi()
{
    int rssicorr;
    if (sx1272) {
        rssicorr = 139;
    } else {
        rssicorr = 157;
    }
    return (readReg(REG_PKT_RSSI_VALUE) - rssicorr);
}

int LoRa::Rssi()
{
    int rssicorr;
    if (sx1272) {
        rssicorr = 139;
    } else {
        rssicorr = 157;
    }
    return (readReg(REG_RSSI_VALUE) - - rssicorr);
}

long int LoRa::packetSnr()
{
    long int SNR;
    byte value = readReg(REG_PKT_SNR_VALUE);
    if( value & 0x80 ) // The SNR sign bit is 1
    {
        // Invert and divide by 4
        value = ( ( ~value + 1 ) & 0xFF ) >> 2;
        SNR = -value;
    } else {
        // Divide by 4
        SNR = ( value & 0xFF ) >> 2;
    }
    return SNR;
}

void LoRa::receivepacket() {
    if(digitalRead(this->_dio0) == 1)
    {
        if(receive(message)) {
            printf("Packet RSSI: %d, ", packetRssi());
            printf("RSSI: %d, ", Rssi());
            printf("SNR: %li, ", packetSnr());
            printf("Length: %i", (int)(this->_receivedbytes));
            printf("\n");
            printf("Payload: %s\n", message);
        } // received a message

    } // dio0=1
}

void LoRa::configPower (int8_t pw) {
    if (sx1272 == false) {
        // no boost used for now
        if(pw >= 17) {
            pw = 15;
        } else if(pw < 2) {
            pw = 2;
        }
        // check board type for BOOST pin
        writeReg(RegPaConfig, (uint8_t)(0x80|(pw&0xf)));
        writeReg(RegPaDac, readReg(RegPaDac)|0x4);

    } else {
        // set PA config (2-17 dBm using PA_BOOST)
        if(pw > 17) {
            pw = 17;
        } else if(pw < 2) {
            pw = 2;
        }
        writeReg(RegPaConfig, (uint8_t)(0x80|(pw-2)));
    }
}


void LoRa::writeBuf(byte addr, byte *value, byte len) {
    unsigned char spibuf[256];
    spibuf[0] = addr | 0x80;
    for (int i = 0; i < len; i++) {
        spibuf[i + 1] = value[i];
    }
    selectreceiver();
    wiringPiSPIDataRW(CHANNEL, spibuf, len + 1);
    unselectreceiver();
}

void LoRa::txlora(byte *frame, byte datalen) {

    // set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
    writeReg(RegDioMapping1, MAP_DIO0_LORA_TXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);
    // clear all radio IRQ flags
    writeReg(REG_IRQ_FLAGS, 0xFF);
    // mask all IRQs but TxDone
    writeReg(REG_IRQ_FLAGS_MASK, ~IRQ_LORA_TXDONE_MASK);

    // initialize the payload size and address pointers
    writeReg(REG_FIFO_TX_BASE_AD, 0x00);
    writeReg(REG_FIFO_ADDR_PTR, 0x00);
    writeReg(REG_PAYLOAD_LENGTH, datalen);

    // download buffer to the radio FIFO
    writeBuf(REG_FIFO, frame, datalen);
    // now we actually start the transmission
    opmode(OPMODE_TX);

    printf("send: %s\n", frame);
}

void LoRa::configTransmitter(){
    opmodeLora();
    opmode(OPMODE_STANDBY);
    writeReg(RegPaRamp, (readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec
    configPower(23);
}

void LoRa::configReceiver(){
    opmodeLora();
    opmode(OPMODE_STANDBY);
    opmode(OPMODE_RX);
}

uint8_t LoRa::getSF(){
    return this->_sf;
}

uint32_t LoRa::getLoRaFreq(){
    return this->_LoRaFreq;
}

byte LoRa::ReceivedBytes() {
    return this->_receivedbytes;
}

void LoRa::dumpRegisters()
{
  for (byte i = 0; i < 128; i++) {
	printf("0x%x : 0x%x\n", i & 0xff, readReg(i) & 0xff);
  }
}

// Stepper constructor
LoRa::LoRa()
{
    setPin();
    setSPIFrequency();
    this->_receivedbytes=0;
}

LoRa::~LoRa()
{
    this->_receivedbytes=0;
}

int LoRa::begin(uint32_t frequency)
{
    if (!isBeginCalled) {
        //printf("Is Called: %i\n",(int)isBeginCalled);
        setPin(this->_ssPin, this->_dio0, this->_RST);
        setSPIFrequency(this->_SPIFreq);
        isBeginCalled = true;

        digitalWrite(this->_RST, HIGH);
        delay(100);
        digitalWrite(this->_RST, LOW);
        delay(100);

        byte version = readReg(REG_VERSION);

        if (version == 0x22) {
            // sx1272
            printf("SX1272 detected, starting.\n");
            sx1272 = true;
        } else {
            digitalWrite(this->_RST, LOW);
            delay(100);
            digitalWrite(this->_RST, HIGH);
            delay(100);
            version = readReg(REG_VERSION);
            if (version == 0x12) {
                // sx1276
                printf("SX1276 detected, starting.\n");
                sx1272 = false;
            } else {
                printf("Unrecognized transceiver.\n");
                printf("Version: 0x%x\n",version);
                exit(1);
            }
        }
        opmode(OPMODE_SLEEP);
        setFrequency(frequency);
        setSyncWord();
        setSpreadingFactor();

        writeReg(REG_MAX_PAYLOAD_LENGTH,0x80);
        writeReg(REG_PAYLOAD_LENGTH,PAYLOAD_LENGTH);
        writeReg(REG_HOP_PERIOD,0xFF);
        writeReg(REG_FIFO_ADDR_PTR, readReg(REG_FIFO_RX_BASE_AD));

        writeReg(REG_LNA, LNA_MAX_GAIN);
            //printf("_ssPin: %i\n", this->_ssPin);
            //printf("_dio0: %i\n", this->_dio0);
            //printf("_RST: %i\n", this->_RST);
            //printf("Freq: %li\n", frequency);
           //printf("SPIFreq: %li\n", this->_SPIFreq);
    } else {
        printf("Call begin() only one time\n");
        printf("Is Called: %i\n",(int)isBeginCalled);
    }
    return 1;
}
