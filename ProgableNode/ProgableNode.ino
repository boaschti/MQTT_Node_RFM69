/*
Author:  Sebastian Weisser
Date 28.01.2017
Modifications Needed:
1)  Update encryption string "ENCRYPTKEY"
2)  Frequency setting
3)	Pubsubclient replace #define MQTT_MAX_PACKET_SIZE 256
4)	Adafruit_BME280.cpp replace "if (read8(BME280_REGISTER_CHIPID) != 0x60)" with "uint8_t chipId = read8(BME280_REGISTER_CHIPID); if ((chipId != 0x58) && (chipId != 0x60))"
5)	RFM69.h replace   #define RF69_IRQ_PIN          3    #define RF69_IRQ_NUM          1


*/


//Basic Defines  --------------------------------------------------------------------------------------------------
#include <RFM69.h>
#include <SPI.h>
#include <avr/sleep.h>
//#include <stdint.h>
#include <avr/wdt.h> 
#include <RFM69registers.h>

//#include <eeprom.h>

//Standardkonfig wird uebernommen wenn JP_2 == GND oder funktion_pin0 == 255 (Komando "w_0":"255")
#define DEFAULTNODEID        250    //unique for each node on same network
#define DEFAULTNETWORKID     146  //the same on all nodes that talk to each other
#define DEFAULTGATEWAYID     1
#define DEFAULTENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!

//Match frequency to the hardware version of the rfm69 on your Moteino (uncomment one):
#define FREQUENCY   RF69_433MHZ
//#define FREQUENCY   RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ACK_TIME		30 // max # of ms to wait for an ack
#define LED_1			5  //optinonal to Power >3V sensors from 1.8V batt voltage: Write 0/1/0/1 to pump Power for external sensors (and switch on LED_2)
#define LED_2			6 //optinonal to Power >3V sensors from 1.8V batt voltage: Power for external Sensors
#define LED_3			7 
#define ResetRfmPin		2
#define rxPollTime		3000 // Zeit in ms (ca) die auf eine Message gewartet wird bevor der Node in den Sleep geht
//you can use the analog pins as digital pins, by numbering them 14 - 19
//Analog Input PC0-PC5
#define JP_1		  16 //PC2 
#define JP_2		  15 //PC1
#define RFM_POWER_LEVEL 31




RFM69 rfm69;



//end Basic Defines ---------------------------------------------------------------------------------------------------


//Board Defines--------------------------------------------------------------------------------------------------------
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <U8x8lib.h>

//U8X8_SSD1306_64X48_ER_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);  
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);  

//device DHT11 Temperature/Humidity
    #define DHTPIN 16     // what pin we're connected to
    #define DHTTYPE DHT11   // DHT 11 
    DHT dht(DHTPIN, DHTTYPE);

// device DS18B20
    #define ONE_WIRE_BUS 9
    OneWire oneWire(ONE_WIRE_BUS);
    DallasTemperature dallas(&oneWire);
    
//BME280 I2C
    #define SEALEVELPRESSURE_HPA (1013.25)
    Adafruit_BME280 bme;
    
//ADC6 atmega voltage, to measure switch on Port 6 
    #define circuitOn		6 //Port6
    #define atVolage		6 //ADC6

//Pins to Power up the Sensors
    #define pumpPin			6
    #define supplyPin		5
    
//device ultrasonic
    #define HC05pingPin 	0
    #define HC05trigPin		1

#define SERIAL_BAUD 9600


unsigned long WatchdogPeriod;
unsigned long SensorPeriod;
volatile unsigned long WdTrigTimeStamp;
unsigned long WdPinTimeout;
volatile boolean SleepAlowed;
boolean BreakSleep = 0;  //todo volatile ???

//end Board Defines ---------------------------------------------------------------------------------------------------


#define configSize 31

uint8_t eeConfig[configSize] EEMEM;
uint8_t config[configSize];
uint8_t eeEncryptKey[16] EEMEM;


//Folgende Definitionen zeigen die Bit stellen der einstellbaren Funktionen.

//Es koennen folgende  Eigenschaften fuer die Pins eingestellt werden:
//Bits der Variablen funktion_pin..
#define in_out			0		    //DDR wie im Atmel Datenblatt 0=in
#define port            1           //PORT wie im Atmel Datenblatt 1=High(outputMode)/1=Pullup(inputMode)
#define pcInt           2           //Der Pin loest einen Interrupt aus (aufwachen aus dem Sleep)
#define pwm             3           //Der Pin gibt ein software PWM Signal aus
#define wdDefault       4           //Zustand in den der watchdog faellt wenn er nicht getriggert wird (nur wenn kein sleep aktiv ist!)
#define wdReq           5           //Watchdog fuer diesen Pin aktivieren
#define readInput       6           //Ruecklesen des Pins und senden der Daten (polling)

//Zum konfigurieren der analog Inputs muss man angeben wie der Wert verrechnet wird.
//Bits der Variablen math_analog..
#define readPlaint      0           //Arduino Pflanzen Feuchte Sensor (untested)
#define readLDR         1           //Photo Widerstand 10k pulldown
#define readRain        2           //Arduino Regen Sensor
#define readRaw        3            //raw adc Wert

//Zum konfigurieren der digitalen Sensoren muss man nur angeben welche Sensoren gelesen werden sollen.
//Bits der Variable digitalsensors
#define readDS18        0           //(Pins s. defines ONE_WIRE_BUS) (Quartz erforderlich)
#define readHC05        1           //Ultraschall Sensor (Pins s. defines HC05pingPin und HC05trigPin) (Quartz erforderlich)
#define readBME         2           //BME280 und BMP280 (Standart I2C Pins)
#define readDHT         3           //(Quartz erforderlich)

//Zum kofigurieren von digitalen Aktoren muss man nur angeben welche Aktoren verbaut sind.
//Bits der Variable digitalOut
#define writeWsLed      0           //Max 255 ws2812b LEDs
#define uart			1
#define dmx             2           //Wenn ein DMX Signal ausgegeben werden soll (Quartz erforderlich)
#define ssd1306_64x48   3
#define ssd1306_128x64  4

//Zum einstellen des Verhaltens der Node:
//Bits der Variable nodeControll
#define sensorPower			0       //der supplyPin (5) wird zum Versorgen von Sensoren und fuer die LED2 verwendet. Wenn 0 dann nur fue die LED2.
#define pumpSensorVoltage   1		//der pumpPin (6) wird zum verdoppeln der Betriebsspannnung fuer Sensoren verwendet. Sowie fuer die LED1. Wenn 0 dann nur fue die LED1.


//Die folgenden Definitionen zeigen die Stelle im BYTE Array wo die Einstellungen gespeichert sind:
//Man kann direkt auf das ARRAY per Funk zugreifen die Werte sind immer dezimal:
//"w_5":"3" -> Zum Schreiben einer 3 an Stelle 5
//"r_5":"3" -> Zum Lesen der Stelle 5 (die "3" wird gebraucht allerdings ignoriert)
//"p_5":"1" -> Zum setzen und ruecksetzen des Ports 5 (funktion_pin..)

//Bytes des Config Arrays
#define  funktion_pin0          0
#define  funktion_pin1          1
#define  funktion_pin17         2
#define  funktion_pin18         3
#define  funktion_pin19         4
#define  funktion_pin9          5
#define  funktion_pin16         6

#define  math_analog2           10
#define  math_analog3           11
#define  math_analog4           12
#define  math_analog5           13
#define  digitalSensors         15
#define  digitalOut             16

#define  nodeControll           20
#define  sleepTimeMulti         21    //sleepTime Multiplikator
#define  sleepTime              22    //Sleep time in Sekunden * 8 sec, Wenn der Timer gesetzt ist wird auch die Batteriespannung gemessen
#define  watchdogTimeout        23    //watchdog in * 8 sec, Zeit bis zum abfallen des Watchdogs und setzen der Pins auf den vorgegebenen Zustand (wdDefault). todo Der Sleep wird gesperrt wenn der Watchdog nicht abgefallen ist.
#define  watchdogDelay          24    //watchdog in * 5 sec, Zeit bis zum nachsten senden eines Watchdogs (Nur wenn sleepTime == 0 konfiguriert ist oder der watchdog durch staendiges Nachtriggern den Sleep blokiert. Ansonsten wird der Watchdog sofort nach dem Aufwachen gesendet wenn watchdogDelay>0)

#define  contrast               26
#define  nodeId                 27    //NodeId dieses Sensors (einzigartig im Netzwerk)
#define  networkId              28    //NetworkID dieses Sensors (alle gleich im Netzwerk)
#define  gatewayId              29    //GatewayID an diese Addresse werden die Daten geschickt und es werden nur Daten von dieser Addresse beruecksichitgt. Todo Sollten Daten von einer Anderen Adresse kommen werden sie an das Gateway weitergeleitet.
#define  sensorDelay            30    //Messpause der Sensoren in * 10 sec (Nur wenn sleepTime == 0 konfiguriert ist oder der watchdog durch staendiges Nachtriggern den Sleep blokiert)

//In den folgenden Definitionen ist das PinMapping beschrieben: fur digtal IOs config[0]:config[9], fur analog Is config[10]:config[14].
#define usedDio                 7       //Anzahl der verwendetetn DIOs
#define usedAnalog				4		//Anzahl der verwendetetn analog In
const uint8_t pinMapping[15]{0,1,17,18,19,9,16,0,0,0,2,3,4,5,0};
//der SSPin macht noch Probleme Spi funktioniert dann nicht mehr!
//const uint8_t pinMapping[15]{0,1,17,18,19,9,10,0,0,0,3,4,5,0,0};

#define bit_set(p,m) ((p) |= (1<<m))
#define bit_clear(p,m) ((p) &= ~(1<<m))
#define bit_write(p,m,c) (c ? bit_set(p,m) : bit_clear(p,m))

#define wdt_set(value)   \
__asm__ __volatile__ (  \
"in __tmp_reg__,__SREG__" "\n\t"    \
"cli" "\n\t"    \
"wdr" "\n\t"    \
"sts %0,%1" "\n\t"  \
"out __SREG__,__tmp_reg__" "\n\t"   \
"sts %0,%2" \
: /* no outputs */  \
: "M" (_SFR_MEM_ADDR(_WD_CONTROL_REG)), \
"r" (_BV(_WD_CHANGE_BIT) | _BV(WDE)), \
"r" ((uint8_t) ((value & 0x08 ? _WD_PS3_MASK : 0x00) | \
_BV(WDIE) | (value & 0x07)) ) \
: "r0"  \
)

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void initVariables(void)
{
    //Standard config laden
    //Alle Pins auf Eingang
    //keine Sensoren aktiv
    //es kann bei der ersten Inbetriebnahme zu Problemen (haengt beim Sensor lesen) kommen wenn diese Variable nicht auf 255 steht
    //if ((eeprom_read_byte(&eeConfig[funktion_pin0]) == 255) || !getJumper()){
    if (eeprom_read_byte(&eeConfig[funktion_pin0]) == 255){
    //if (1){
        for (uint8_t i = 0; i < configSize-5; i++){
            eeprom_write_byte(&eeConfig[i], 0);
        }
        eeprom_write_byte(&eeConfig[contrast], 255);        
        eeprom_write_byte(&eeConfig[nodeId], DEFAULTNODEID);
        eeprom_write_byte(&eeConfig[networkId], DEFAULTNETWORKID);
        eeprom_write_byte(&eeConfig[gatewayId], DEFAULTGATEWAYID);
        eeprom_write_byte(&eeConfig[sensorDelay], 2);	
        //eeprom_write_block(eeEncryptKey, DEFAULTENCRYPTKEY, 16);
    }
    
    for (uint8_t i = 0; i < configSize; i++){
        config[i] = eeprom_read_byte(&eeConfig[i]);
    }
    
    SensorPeriod = config[sensorDelay] * 10000;
    WatchdogPeriod = config[watchdogDelay] * 5000;
    WdPinTimeout = config[watchdogTimeout] * 5000;
    //Wir wollen den SLeep erst frei geben wenn der Watchdog abgelaufen ist
    if (WdPinTimeout > 0){
        SleepAlowed = false;
    }else{
        SleepAlowed = true;
    }
    
}

void check_improveConfig(void){
        
        
            
}

void setupPins(void)
{
    // Onboard LEDs
    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);
    pinMode(LED_3, OUTPUT);
    
    //reset Pin Rfm
    pinMode(ResetRfmPin, OUTPUT);
    
    //starte ADC for readPlaint oder readLDR oder readRain
    if (config[math_analog2] || config[math_analog3] || config[math_analog4] || config[math_analog5]){
        //analogReference(INTERNAL);
        analogReference(EXTERNAL);
    }
    
    //Config DIOs
    for (uint8_t i = 0; i < usedDio; i++){
        //der SSPin macht noch Probleme Spi funktioniert dann nicht mehr!
        if (pinMapping[i] != 10){
            pinMode(pinMapping[i], (config[i] & (1<<in_out)));
            digitalWrite(pinMapping[i], (config[i] & (1<<port)));
        }
        if (config[i] & (1 << pcInt)){	
            pciSetup(pinMapping[i]);
        }
    }	

    //Wenn eine SleepTime configuriert ist gehen wir davon aus dass die Batteriespanung auch gelesen werden soll
    if (config[sleepTime] > 0){
        pinMode(circuitOn, OUTPUT);
    }
    
    if (config[digitalSensors] & (1<<readHC05)){
        pinMode(HC05pingPin, INPUT);
        pinMode(HC05trigPin, OUTPUT);
    }
            
}

uint8_t getJumper(void){
    //0=GND
    //1=VDD
    //2=open
        pinMode(JP_2, INPUT);
        delay(5);
        if (digitalRead(JP_2)){
            return 1;
        }	
        pinMode(JP_2, INPUT_PULLUP);
        delay(5);
        if (digitalRead(JP_2)){
            return 2;
        }else{
            return 0;
        }
}

void disableWd(void){
    //Reset WD Timer
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    WDTCSR = 0;
}

void setup()
{	
    //disable Watchdog Timer
    disableWd();

    initVariables();
    check_improveConfig();
    setupPins();
    //reset_wdPins(); 
    
    //RFM69-------------------------------------------
    //Reset

    digitalWrite(ResetRfmPin, HIGH);
    delay(100);
    digitalWrite(ResetRfmPin, LOW);
    delay(100);
    //Init RFM69
    rfm69.initialize(FREQUENCY, config[nodeId], config[networkId]);
    //rfm69.initialize(FREQUENCY,NODEID,NETWORKID);
    #ifdef IS_RFM69HW
        rfm69.setHighPower(); 
    #endif
    rfm69.setPowerLevel(RFM_POWER_LEVEL);
    //char temp[16];
    //eeprom_read_block(temp, eeEncryptKey, 16);
    //rfm69.encrypt(&temp[0]);
    rfm69.encrypt(DEFAULTENCRYPTKEY);

    rfm69.receiveDone();		//goto Rx Mode
  
    //--------------------------------------
    
    //ssd1306 Display
    //u8x8.setI2CAddress(0x3c);
    if ((config[digitalOut] & (1<<ssd1306_128x64)) || (config[digitalOut] & (1<<ssd1306_64x48))){
        u8x8.begin();
        u8x8.setPowerSave(0);
        u8x8.setContrast(config[contrast]);
        u8x8.setFont(u8x8_font_pxplusibmcga_r);
        u8x8.drawString(4,2,"Node");
        u8x8.drawString(4,3,"started");
    }

    //device DHT
    if (config[digitalSensors] & (1<<readDHT)){
        dht.begin();
    }
    
    //device UART
    if (config[nodeControll] & (1<<uart)){
        Serial.begin(SERIAL_BAUD);
    }
    
    //device DS18B20
    if (config[digitalSensors] & (1<<readDS18)){
        dallas.begin();
    }
    
    if (config[digitalSensors] & (1<<readBME)){
        if (!bme.begin()) {
            const char errorString[] = "\"err\":\"BME Init\"";
            rfm69.sendWithRetry(config[gatewayId], errorString, sizeof(errorString));
        }
    }

    const char errorString[] = "\"info\":\"setup_Node\"";
    rfm69.sendWithRetry(config[gatewayId], errorString, sizeof(errorString));
}

 boolean write_buffer_str(char *name, char *wert, boolean strWert = false )
{
    /*
    write_buffer_str("abc","123") -> Es wird ein String '"abc":123' in den Buffer geschrieben
    write_buffer_str("abc","123",false) -> Es wird ein String '"abc":"def"' in den Buffer geschrieben	
    write_buffer_str("","") -> Es werden alle Daten gesendet und der Pointer auf 0 gesetzt
    
    Es wird immer ueberprueft ob die maxixmale laenge von 61Bytes (begrenzt vom RFM Modul) ueberschritten wird 
    Es wird immer ueberprueft ob die neue Nachricht + messageOverhead noch Platz hat. Wenn nicht wird erst gesendet
    */
    
    #define messageOverheadStr 6	//"":"",
    #define messageOverheadDec 4	//"":,
    
    static char sendBuffer[RF69_MAX_DATA_LEN];
    static uint8_t sendBufferPointer = 0;	
    uint8_t nameLength = strnlen(name, RF69_MAX_DATA_LEN);
    uint8_t wertLength = strlen(wert);
    uint8_t messageOverhead;
    
    if (strWert == true){
        messageOverhead = messageOverheadDec;
    }else{
        messageOverhead = messageOverheadStr;
    }
    
    //Es wird geprueft ob die Nachricht ueberhaupt in den Buffer passt ansonsten senden wir eine Fehlermeldung
    if ((nameLength + wertLength + messageOverhead) <= RF69_MAX_DATA_LEN)
    {
        //Wir pruefen ob die Nachricht zusaetlich in den Buffer passt ansonsten schicken wir zuerst alle Daten weg
        //Wenn die wertLange 0 ist, und Daten, die noch nicht gesendet worden sind vorliegen, dann wollen wir diese senden 
        if (((nameLength + wertLength + sendBufferPointer + messageOverhead) > RF69_MAX_DATA_LEN) || ((wertLength == 0) && (sendBufferPointer > 0))){
            if (!rfm69.sendWithRetry(config[gatewayId], sendBuffer, sendBufferPointer)){
                //Wir konnten nicht senden-> wir warten und probieren es noch einmal
                delay(config[nodeId]*1.3);
                if (!rfm69.sendWithRetry(config[gatewayId], sendBuffer, sendBufferPointer)){
                    //Ein Fehler ist aufgetreten wir merken uns den Bufferinhalt
                    radio_Rx_loop(); //RFM->RXMode
                    return false;
                }else{
                    sendBufferPointer = 0;
                }
            }else{
                sendBufferPointer = 0;
            }
        }
        // WIr wollen nur in den Puffer schreiben wenn auch ein Wert uebergeben wurde
        if (wertLength != 0){
            sendBuffer[sendBufferPointer++] = '\"';

            for (uint8_t loop = 0; name[loop] != '\0'; loop++)
            {
                sendBuffer[sendBufferPointer++] = name[loop];
            }

            sendBuffer[sendBufferPointer++] = '\"';
            sendBuffer[sendBufferPointer++] = ':';
            if (strWert == true){
                sendBuffer[sendBufferPointer++] = '\"';
            }

            for (uint8_t loop = 0; loop < wertLength; loop++)
            {
                sendBuffer[sendBufferPointer++] = ((char*)(wert))[loop];
            }

            if (strWert == true){
                sendBuffer[sendBufferPointer++] = '\"';
            }
            sendBuffer[sendBufferPointer++] = ',';
            sendBuffer[sendBufferPointer] = '\0';
        }
    }
    else
    {
        const char errorString[] = "\"err\":\"Message to long\"";
        rfm69.sendWithRetry(config[gatewayId], errorString, sizeof(errorString));
        radio_Rx_loop(); //RFM->RXMode
        return false;
    }
    
    radio_Rx_loop(); //RFM->RXMode
    return true;
}

void sendInt(char *name, uint8_t wert){
    
    char temp[5];
    itoa(wert, temp, 10);
    write_buffer_str(name, &temp[0]);
    write_buffer_str("",""); //sende Daten
}

boolean readMessage(char *message){
    // "R_12":"125"
    char parts[15][10];
    char *p_start, *p_end;
    uint8_t i = 0;
    p_start = message;
    while(1) {
        p_end = strchr(p_start, '_'); 
        if (p_end) {							//copy R
            strncpy(parts[i], p_start+1, p_end-p_start-1);
            parts[i][p_end-p_start-1] = 0;
            i++;
            p_start = p_end + 1;
            p_end = strchr(p_start, '/"'); 
            if (p_end) {						//copy12
                strncpy(parts[i], p_start, p_end-p_start);
                parts[i][p_end-p_start] = 0;
                i++;
                p_start = p_end + 1;
                p_end = strchr(p_start, '/"');
                if (p_end) {					//prepare to search next "
                    p_start = p_end + 1;
                    p_end = strchr(p_start, '/"');
                    if (p_end) {				//copy 125
                        strncpy(parts[i], p_start, p_end-p_start);
                        parts[i][p_end-p_start] = 0;
                        i++;
                        p_start = p_end + 1;
                        p_end = strchr(p_start, '/"');
                        if (p_end) {			//search next " to set pointer
                            p_start = p_end;
                        }
                    }
                }
            }
        }
        else {
            // copy the last bit - might as well copy 20
            strncpy(parts[i], p_start, 20);
            break;
        }
    }

    //parts[0]
    //parts[1]
    // last part parts[i]
    /*
        write_buffer_str("part0", parts[0], true);
        write_buffer_str("part1", parts[1], false);
        write_buffer_str("part2", parts[2], false);
        write_buffer_str("part3", parts[3], false);
        write_buffer_str("part4", parts[4], false);
        write_buffer_str("part5", parts[5], false);
        write_buffer_str("part6", parts[6], false);
        write_buffer_str("part7", parts[7], false);
        write_buffer_str("part8", parts[8], false);
        write_buffer_str("part9", parts[9], false);
        rfm69.sendWithRetry(config[gatewayId], sendBuffer, sendBufferLength);
        */
    uint8_t listLen = i;
    boolean resetCPU = false;
    
    for (uint8_t i=0; i<=listLen; i++){
        /*
        write_buffer_str("parse", parts[i], true);
        rfm69.sendWithRetry(config[gatewayId], sendBuffer, sendBufferLength);
        delay(100);
        */
        //zum schreiben einer festen config
        if (strcmp(parts[i] , "w") == 0){
            if (atoi(parts[i + 1]) < configSize){
                eeprom_write_byte(&eeConfig[atoi(parts[i + 1])], atoi(parts[i + 2]));
                config[atoi(parts[i + 1])] = eeprom_read_byte(&eeConfig[atoi(parts[i + 1])]);
                char temp[10] = "infoReg";
                strncat(temp, parts[i+1],2);
                sendInt(temp, config[atoi(parts[i + 1])]);
                i++;
                i++;
                //Wir wollen in einem sauberen Zustand starten
                //todo if funktoniert so nicht
                if ((atoi(parts[i + 1]) != contrast)){
                    resetCPU = true;
                }else{
                    setup();
                }
            }
        }
        //zum lesen der Config
        if (strcmp(parts[i] , "r") == 0){
            char temp[10] = "infoReg";
            strncat(temp, parts[i+1],2);
            sendInt(temp, config[atoi(parts[i + 1])]);
            i++;
            i++;
        }
        //zum setzen von Ports
        if (strcmp(parts[i] , "p") == 0){
            //Wenn fuer diesen Pin ein Watchdog aktiv ist dann darf der sleep nich tausgefuehrt werden
            if (config[atoi(parts[i + 1])] & (1<<wdReq)){
                SleepAlowed = false;	
            }
            //todo Ports nicht setzen wenn Watchdog abgelaufen ist
            if (strcmp(parts[i+2] , "1") == 0){
                digitalWrite(pinMapping[atoi(parts[i + 1])],HIGH);
            }else{
                digitalWrite(pinMapping[atoi(parts[i + 1])],LOW);
            }
            //setze Bit "readInput" fuer den jeweiligen Port damit der Status zurueck gesendet wird
            i++;
            i++;
        }
        if (strcmp(parts[i] , "d") == 0){
            if (config[digitalOut] & (1<<ssd1306_64x48)){
                u8x8.drawString(4, (atoi(parts[i + 1])+2), "        ");
                u8x8.drawString(4, (atoi(parts[i + 1])+2), parts[i + 2]);
            }else if (config[digitalOut] & (1<<ssd1306_128x64)){
                u8x8.drawString(0, atoi(parts[i + 1]), "            ");
                u8x8.drawString(0, atoi(parts[i + 1]), parts[i + 2]);
            }
        }
        //den Watchdog nachtriggern
        if (strcmp(parts[i] , "wd") == 0){
            WdTrigTimeStamp = millis();
            SleepAlowed = false;		
            i++;
            i++;
        }
    }
    if (resetCPU){
        write_buffer_str("info", "restart_Node", true);
        write_buffer_str("","");	
        WDTCSR |= (1<<WDCE) | (1<<WDE);
        WDTCSR |= (1<<WDE) | (1<<WDP1) | (1<<WDP2);
        while(1);
    }
}

void radio_Rx_loop(void) {
    //pruefen ob daten vom RFM vorliegen
    if (rfm69.receiveDone())
    {
        uint8_t senderId;
        int16_t rssi;
        uint8_t data[RF69_MAX_DATA_LEN];
        uint8_t dataLen;
        
        digitalWrite(LED_1, HIGH);
        
        //speichern der Daten, da evt schon wieder welche ankommen
        senderId = rfm69.SENDERID;
        rssi = rfm69.RSSI;
        dataLen = rfm69.DATALEN;
        memcpy(data, (void *)rfm69.DATA, dataLen);
        //check if sender req an ACK
        if (rfm69.ACKRequested())
        {
            rfm69.sendACK();
        }
        rfm69.receiveDone(); //put rfm69 in RX mode
        if (senderId == config[gatewayId]){
            //ergaenze NULL Terminierung
            data[dataLen] = 0;	
            readMessage((const char *)data);
        }else{
            //Wir wollen Daten von einer anderen Node mit deren Namen versenden, wenn diese die Addresse als GATEWAYID eingestellt hat
            //rfm69.initialize(FREQUENCY, senderId, config[networkId]);
            rfm69.sendWithRetry(config[gatewayId], data, dataLen);
            //rfm69.initialize(FREQUENCY, config[nodeId], config[networkId]);
        }
        digitalWrite(LED_1, LOW);

        } 
    else {
            rfm69.receiveDone(); //put rfm69 in RX mode
    }
}

void read_bme(void){
    
    float temp_F;
    char Temp[10];
    temp_F = bme.readTemperature();
    //dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf);
    dtostrf(temp_F, 3, 1, Temp);
    write_buffer_str("Bt", &Temp[0]);
    
    temp_F = bme.readPressure() / 100.0F;				//in hPa
    //dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf);
    dtostrf(temp_F, 3, 1, Temp);
    write_buffer_str("Bp", &Temp[0]);
    
    temp_F = bme.readAltitude(SEALEVELPRESSURE_HPA);	//Hoehe in m
    //dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf);
    dtostrf(temp_F, 3, 1, Temp);
    write_buffer_str("Ba", &Temp[0]);
        
    //Nur bei BME280 nicht bei BMP280
    temp_F = bme.readHumidity();
    //dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf);
    dtostrf(temp_F, 3, 1, Temp);
    write_buffer_str("Bh", &Temp[0]);	
        
}

void read_Dallas(void)
{
    
    //Nach dem einschalten des Sensors muss man min 1sec warten bis der Sensor richtige Daten liefert. Das muss der Aufrufer garantieren.

    float temp_F;
    
    for (uint8_t i = 0; i < dallas.getDeviceCount(); i++){
        char temp[7] = "Dt";
        temp_F = dallas.getTempCByIndex(i);
        //dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf);
        char wert[6];
        dtostrf(temp_F, 3, 1, wert);
        char *dsAddr;
        dallas.getAddress(dsAddr, i);
        strncat(temp,dsAddr,3); //Wir wollen nur 3 Stellen der Addresse senden
        write_buffer_str(&temp[0], &wert[0]);

    }
}


void read_DHT(boolean readImmediatelly = false)
{
    
    //Nach dem einschalten des Sensors muss man min 1sec warten bis der Sensor richtige Daten liefert. Das muss der Aufrufer garantieren.
    
    //float h = dht.readHumidity();
    //float t = dht.readTemperature();
    float h = 2.1;
    float t = 30.12;
        
    float temp_F;
    if (isnan(t) || isnan(h)) {
        write_buffer_str("err", "DHT_read",true);
        write_buffer_str("","");
    } else {
        char Temp[10];
        //dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf);			
        dtostrf(h, 1, 0, Temp);
        write_buffer_str("DHh", &Temp[0]);
        temp_F = t * 9/5+32;			
        dtostrf(temp_F, 3, 1, Temp);
        write_buffer_str("DHt", &Temp[0]);
    }
}

void read_HC05(void){
    //cm = microseconds / 29 / 2;
    write_buffer_str("funk", "HC05 read",true);
}

uint16_t read_Vcc(boolean sendWert = true){
    
    // Lese 1.1V reference gegen AVcc

    uint16_t result;   
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2);
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA,ADSC));
    result = ADCL;
    result |= ADCH<<8;	
    result = 1126400 / result; //1024 * Vbandgap(1100mV) / ADC_Wert
    //Wir brauchen die Funktion auch fuer interne Zwecke
    if (sendWert){
        char temp[10];
        itoa(result, temp, 10);
        write_buffer_str("batt" , temp);
    }
    return result;
}

void read_analog(void){
    
    for (uint8_t i = 10; i < (usedAnalog + 10); i++){
        if (config[i] != 0){
            char temp[6] = "Ai";
            char tempindex[3];
            char wert[7];
            itoa(pinMapping[i], tempindex, 10);
            strncat(temp, tempindex, 2);
            uint16_t OutputWert;
            OutputWert = analogRead(pinMapping[i]);
            if (config[i] & (1<<readPlaint)){
                
            }else if (config[i] & (1<<readRain)){
                
            }else if (config[i] & (1<<readLDR)){
                // GND----[_10K_]--+--LDR----VDD
                //                 |________ADin
                //Wir muessen nichts berechnen
                
            }else if (config[i] & (1<<readRaw)){
                //Wir muessen nichts berechnen
            }
            itoa(OutputWert,wert,10);
            write_buffer_str(temp,wert);
        }
        
    }
    
    
}

void go_sleep(void){
    
    BreakSleep = false;

    digitalWrite(LED_2, LOW);
    
    rfm69.sleep();
    
    u8x8.setPowerSave(1);
    //todo? reg PRR und ADMUX
    
    wdt_set(WDTO_8S);

    for (uint16_t i = 0; i < (config[sleepTime] * config[sleepTimeMulti]); i++){
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here	    
        sleep_enable();         // enables the sleep bit in the mcucr register
        
        sei();
        
        sleep_mode();           // here the device is actually put to sleep!!

        if (BreakSleep){		//Sleep untebrechen wenn ein Interrupt von einem Eingang kam
            break;
        }
        
    }
    
    disableWd();

    //rfm69.receiveDone();	//set RFM to RX
    
    u8x8.setPowerSave(0);
    digitalWrite(LED_2, HIGH);
}

boolean read_inputs(void){
    static boolean readAllInputs = true;
    static uint8_t lastPinState = 0;
    
    //#define bit_write(p,m,c) (c ? bit_set(p,m) : bit_clear(p,m))
    
    for (uint8_t i = 0; i<usedDio; i++){
        if (config[i] & (1<<readInput)){
            boolean inputState;
            inputState = digitalRead(pinMapping[i]) == HIGH;
            if ((inputState != ((lastPinState & (1 << i)) != 0)) || readAllInputs){
                char temp[5] = "IN";
                char tempindex[3];
                char wert[3];
                itoa(pinMapping[i], tempindex, 10);
                strncat(temp, tempindex, 2);
                itoa(inputState,wert,10);
                //Wir wollen uns den aktuellen State nur merken wenn das senden geklappt hat (In der Hoffung dass der Pegel dann noch anliegt)
                if (write_buffer_str(temp, wert)){
                    bit_write(lastPinState, i, inputState);
                }
            }
        }
    }	
    readAllInputs = false;
    return 0;
}

void reset_wdPins(void){
    
    for (uint8_t i = 0; i<usedDio; i++){
        if (config[i] & (1<<wdReq)){
            if (config[i] & (1<<wdDefault)){
                digitalWrite(pinMapping[i], HIGH);
            }else{
                digitalWrite(pinMapping[i], LOW);
            }
        }
    }
    
}

//---------------------------------------------------------------------------------------------
void loop()
{  
    long timepassed;

    static long sensorTimeOld;
    static long watchdogTimeOld;
    static boolean sensorenLesen = true;
    static boolean wdSenden = false;
    
    digitalWrite(LED_3, LOW);

    timepassed = millis() - sensorTimeOld;
    //Wir wollen die Sensoren abfragen wenn die Pausezeit vorbei ist, wenn sensorenLesen gesetzt ist, dann lesen wir sofort
    if ((timepassed > SensorPeriod) || (sensorenLesen == true)){
        //wartezeit bis die Sensoren gelesen werden koennen (DS18b20) sensorenLesen ist nur gesetzt wenn wir aus dem Sleep kommen oder die CPU neu gestartet wurde
        if ((sensorenLesen == true) && (config[digitalSensors] & (1<<readDS18))){ 
            delay(1000);
        }
        sensorenLesen = false;
        sensorTimeOld = millis();
        //Wenn eine SleepTime configuriert ist gehen wir davon aus dass die Batteriespanung auch gelesen werden soll
        if (config[sleepTime] > 0){
            read_Vcc();
        }
        if (config[digitalSensors] & (1<<readDS18)){	
            read_Dallas();
        }
        if (config[digitalSensors] & (1<<readHC05)){
            read_HC05();
        }
        if (config[digitalSensors] & (1<<readBME)){
            read_bme();
        }
        if (config[digitalSensors] & (1<<readDHT)){
            read_DHT();
        }		
        read_analog();
    }
    
    //Lesen der digitalen Inputs
    read_inputs();
    
    //Watchdog
    timepassed = millis() - watchdogTimeOld;
    if (((timepassed > WatchdogPeriod) || (wdSenden == true)) && (WatchdogPeriod > 0)){
        watchdogTimeOld = millis();
        write_buffer_str("wd", "WD_MSG",true);
    }

    //Zum leeren des Buffers und senden aller Daten
    write_buffer_str("","");


    timepassed = millis() - WdTrigTimeStamp;
    if (timepassed > WdPinTimeout){
        //WdTrigTimeStamp = millis();
        reset_wdPins();
        SleepAlowed = true;
    }



    if((config[sleepTime] > 0) && (config[sleepTimeMulti] > 0) && SleepAlowed){
        //Wir senden eine dec11 damit das Gateway diese Node beim Broker subscibed und wir retained Messages erhalten
        const char temp[] = {17};
        rfm69.sendWithRetry(config[gatewayId], temp, sizeof(temp));
        //Wir pruefen bis der Timeout abgelaufen ist ob noch eine Nachricht kommt
        for (uint16_t i = 0; i <= rxPollTime; i++){
            radio_Rx_loop();
            delay(1);
        }
        const char temp2[] = {18};
        rfm69.sendWithRetry(config[gatewayId], temp2, sizeof(temp2));
        //Wenn der Watchdog im letzten Schritt nicht getriggert wurde dann rufen wir den Sleep auf
        if (SleepAlowed){
            //Zum leeren des Buffers und senden aller Daten vor den Sleep
            write_buffer_str("","");
            digitalWrite(LED_3, LOW);
            go_sleep();
            sensorenLesen = true;	//Wir wollen, dass die Sensoren sofort gelesen werden
            wdSenden = true;		//Wir wollen, dass der Watchdog sofort gesendet wird
        }
    }

    //Zum leeren des Buffers und senden aller Daten
    write_buffer_str("","");
    digitalWrite(LED_3, HIGH);
    
}//end loop

ISR (PCINT0_vect)
{
    sleep_disable();        // first thing after waking from sleep:
    read_inputs();
    BreakSleep = true;
}

ISR (PCINT1_vect)
{
    sleep_disable();        // first thing after waking from sleep:
    read_inputs();
    BreakSleep = true;
}

ISR (PCINT2_vect)
{
    sleep_disable();        // first thing after waking from sleep:
    read_inputs();
    BreakSleep = true;
}

ISR (WDT_vect)
{
    sleep_disable();        // first thing after waking from sleep:
}