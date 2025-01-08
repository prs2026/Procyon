#if !defined(LCDLIB)
#define LCDLIB
// core libs
#include <Arduino.h>
#include <macros.h>

SX126x Lora;

class RADIO{


    uint8_t radiochannel = 68;

    const long frequency = 916e6;  // LoRa Frequency

    uint8_t sf = 7;                                                     // LoRa spreading factor: 7
    uint32_t bw = 125000;                                               // Bandwidth: 125 kHz
    uint8_t cr = 5;    

    public:

    int init();
    telepacket recivepacket();

};

int RADIO::init(){
    Serial.println("radio init start");
    SPI.setRX(MISO);
    SPI.setTX(MOSI);
    SPI.setSCK(SCLK);
    Lora.setSPI(SPI);
    Lora.setPins(SXCS,SXRST,BUSY,-1,-1,-1);
    if(!Lora.begin()){
        Serial.println("lora init fail, cry");
    }
    Serial.println("radio init ok");
    Lora.setFrequency(frequency);
    Lora.setTxPower(17,SX126X_TX_POWER_SX1261);
    Lora.setRxGain(LORA_RX_GAIN_BOOSTED);
    Lora.setLoRaModulation(sf, bw, cr,false);

    uint8_t headerType = LORA_HEADER_EXPLICIT;                        // Explicit header mode
    uint16_t preambleLength = 12;                                       // Set preamble length to 12
    uint8_t payloadLength = 16;                                         // Initialize payloadLength to 15
    bool crcType = true;                                                // Set CRC enable
    Lora.setLoRaPacket(headerType, preambleLength, payloadLength, crcType,false);
    Lora.setSyncWord(0x3444);
    return 0;
}

telepacket RADIO::recivepacket(){
    telepacket result;
    

    uint8_t buff[128] = {0,12}; 
    int j = 0;
    delay(100);
    //Serial.printf("avalible: %d\n",Lora.available());
    while (Lora.available() > 0 && j < 127)
    {
        buff[j] = Lora.read();
        j++;
    }
    int k = 0;
    while (buff[k] != 0x12 && k < 127)
    {
        Serial.printf("packet not aligned, expected 0x12 got %x\n",buff[k]);
        k++;
    }
    if (k >= 127)
    {
        Serial.println("bad packet");
        return result;
    }
    
    for (int i = 0; i < sizeof(result.data); i++)
    {
        result.data[k] = buff[k];
        //Serial.printf("%d ",result.data[k]);
        //Serial.printf("%d, ",buff[k]);
        k++;
    }
    

    return result;
}



#endif // LCDLIB