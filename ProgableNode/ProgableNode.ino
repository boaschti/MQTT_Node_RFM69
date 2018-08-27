/*
Author:  Sebastian Weisser
Date 28.01.2017
Modifications Needed:
1)  Update encryption string "ENCRYPTKEY"
2)  Frequency setting
3)	Add Programmer ..\Arduino\hardware\arduino\avr\programmers.txt 
4)	..\Users\..\Documents\Arduino\libraries\Adafruit_BME280_Library\Adafruit_BME280.cpp replace "if (read8(BME280_REGISTER_CHIPID) != 0x60)" with "uint8_t chipId = read8(BME280_REGISTER_CHIPID); if ((chipId != 0x58) && (chipId != 0x60))"
5)	RFM69.h replace   #define RF69_IRQ_PIN          3    #define RF69_IRQ_NUM          1
6)  Add special board defines to ..\Arduino\hardware\arduino\avr\boards.txt
7)  Add: 
        void Adafruit_BME280::sleepMode(void){
          uint8_t oldBME280_REGISTER_CONTROL = read8(BME280_REGISTER_CONTROL);
          oldBME280_REGISTER_CONTROL &= ~((1<<0) | (1<<1));
          write8(BME280_REGISTER_CONTROL, oldBME280_REGISTER_CONTROL);
        }

        void Adafruit_BME280::normalMode(void){
          uint8_t oldBME280_REGISTER_CONTROL = read8(BME280_REGISTER_CONTROL);
          oldBME280_REGISTER_CONTROL |= (1<<0) | (1<<1);
          write8(BME280_REGISTER_CONTROL, oldBME280_REGISTER_CONTROL);
        }

    to ...\Documents\Arduino\libraries\Adafruit_BME280_Library\Adafruit_BME280.cpp
8)  Add 
        void  sleepMode(void);
        void  normalMode(void);
    to ...\Documents\Arduino\libraries\Adafruit_BME280_Library\Adafruit_BME280.h



*/


//Basic Defines  --------------------------------------------------------------------------------------------------
#include <RFM69.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/wdt.h> 
#include <RFM69registers.h>
#include <avr/power.h>

//Standardkonfig wird uebernommen wenn JP_2 == GND oder funktion_pin0 == 255 (Komando "w_0":"255")
#define DEFAULTNODEID        254                    //nur einmal im Netzwerk vorhanden
#define DEFAULTNETWORKID     1                      //bei allen Nodes im Netzwerk gleich
#define DEFAULTGATEWAYID     1      
#define DEFAULTENCRYPTKEY    "sampleEncryptKey"     //exakt 16 Zeichen, gleich auf allen Nodes in diesem Netzwerk

//Frequenz vom RFM Modul
#define FREQUENCY   RF69_433MHZ
//#define FREQUENCY   RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
//#define IS_RFM69HW    //uncomment only for RFM69HW!
#define ACK_TIME		30 // Wartezeit auf einen ACK

#define ResetRfmPin		2
#define rxPollTime		450           // Zeit in ms (ca) die auf eine Message gewartet wird bevor der Node in den Sleep geht
#define timemultiplierWD 5000UL       // ms 
#define timemultiplierSensor 10000UL  // ms 

#define JP_1		  16 //PC2 
#define JP_2		  15 //PC1
#define RFM_POWER_LEVEL 31




RFM69 rfm69;



//end Basic Defines ---------------------------------------------------------------------------------------------------


//Board Defines--------------------------------------------------------------------------------------------------------
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <U8x8lib.h>

#define F_CPU 4000000UL


//U8X8_SSD1306_64X48_ER_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);  
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);  

//device DS18B20
    #define ONE_WIRE_BUS 9
    OneWire oneWire(ONE_WIRE_BUS);
    DallasTemperature dallas(&oneWire);
    
//BME280 I2C
    #define SEALEVELPRESSURE_HPA (1013.25)
    Adafruit_BME280 bme;
    
//Pins zum versorgen der Sensoren
    #define pumpPin			5
    #define supplyPin		6
    
//Pins zum bedienen des Heiz Thermostat Stellrades
    #define signalA         0
    #define signalB         1    
   

//device ultrasonic
    #define HC05pingPin 	0
    #define HC05trigPin		1

#define SERIAL_BAUD 9600


unsigned long WatchdogPeriod;
unsigned long SensorPeriod;
volatile unsigned long WdTrigTimeStamp;
unsigned long WdPinTimeout;
volatile boolean SleepAlowed;
volatile boolean BreakSleep = false;
volatile boolean PCinterrupt = false;
volatile boolean InputAlredyRead = false;

volatile boolean SendAlowed = true;

volatile boolean ThermostatPresent = false;
volatile boolean ShowLeds = false;

bool wdMsgSend = false;

//end Board Defines ---------------------------------------------------------------------------------------------------


#define configSize 31
#define configSizeInternal 2
#define configSizeAll configSize + configSizeInternal

uint8_t eeConfig[configSizeAll] EEMEM;
volatile uint8_t config[configSizeAll];
uint8_t eeEncryptKey[16+1] EEMEM;


//Folgende Definitionen zeigen die Bit stellen der einstellbaren Funktionen.
//Die Standard Einstellungen koennen mit dem setzen von config[0] hergestellt werden.
//Es koennen folgende  Eigenschaften fuer die Pins eingestellt werden:

//Bits der Variablen funktion_pin..
#define in_out          0           //DDR wie im Atmel Datenblatt 0=in
#define port            1           //PORT wie im Atmel Datenblatt 1=High(im outputMode)/1=Pullup(im inputMode) sowie zustand in den der Port bei ablaufen des WD faellt
#define intRise         2           //Der Pin loest einen Interrupt aus (aufwachen aus dem Sleep und sofortiges lesen des Pins wenn readInput gesetzt ist)
#define intFall         3           //todo Der Pin gibt ein software PWM Signal aus
#define free            4           
#define wdReq           5           //Watchdog fuer diesen Pin aktivieren
#define readInput       6           //Ruecklesen des Pins und senden der Daten (polling fuer Interrupt pcInt setzen)
#define count           7           //Wenn die anzahl der Schaltvorgaenge gezaehlt werden soll

//Zum konfigurieren der analog Inputs muss man angeben wie der Wert verrechnet wird.
//Bits der Variablen math_analog..
#define readPlant       0           //Arduino Pflanzen Feuchte Sensor (untested)
#define readLDR         1           //Photo Widerstand 10k pulldown
#define readRain        2           //Arduino Regen Sensor
#define readRaw         3           //raw adc Wert
#define readVolt        4           //reads voltage of a ADC
#define multi2         6            //multiply ADC / 2 
#define multi4         7            //multiply ADC / 4 

//Zum konfigurieren der digitalen Sensoren muss man nur angeben welche Sensoren gelesen werden sollen.
//Bits der Variable digitalsensors
#define readDS18        0           //(Pins s. defines ONE_WIRE_BUS)
#define readHC05        1           //Ultraschall Sensor (Pins s. defines HC05pingPin und HC05trigPin)
#define readBME         2           //BME280 und BMP280 (Standart I2C Pins)


//Zum kofigurieren von digitalen Aktoren muss man nur angeben welche Aktoren verbaut sind.
//Bits der Variable digitalOut
#define lockThermostate 0           //Lock the encoder of a thermostate
#define uart            1
#define free2           2           
#define ssd1306_64x48   3           //WEMOS Schield mit ssd1306 und 64x48 Pixel, SCL:D1 SDA:D2 VDD:3V3 GND:GND
#define ssd1306_128x64  4           //Display Board mit ssd1306 und 128x64 Pixel, Board mit 7 Pins: R1,R4,R6,R7:4,7k R8:0 SCL:D0 SDA:D1 VDD:3V3 GND:GND,DC RES*
//     100nF 10k
//GND---||---[__]---VDD
//         |________RES*

//Zum einstellen des Verhaltens der Node:
//Bits der Variable nodeControll
#define sensorPower         0       //der supplyPin (5) wird zum Versorgen von Sensoren und fuer die LED2 verwendet. Wenn 0 dann nur fue die LED2.
#define pumpSensorVoltage   1       //der pumpPin (6) wird zum verdoppeln der Betriebsspannnung fuer Sensoren verwendet indem ein Kondensator aufgeladen wird. Sowie fuer die LED1. Wenn 0 dann nur fue die LED1.
#define sensorPowerSleep    2       //die Sensor versorgung wird waehrend dem Sleep nicht abgeschaltet. Wenn pumpSensorVoltage gesetzt ist wird alle 8sec der Kondensator erneut aufgeladen.
#define debugLed            3       //schaltet die Led Status anzeigen 
#define displayAlwaysOn     4       //Das Display wird im sleep nicht abgeschaltet
#define delOldData          5       //Die alten Daten die evtl nicht gesendet wurden werden in einem neuen Aufwachzyklus geloescht
#define sendAgain           6       //Die alten Daten die evtl nicht gesendet wurden werden nach einem wd Zyklus (8s) erneut gesendet

//Einstellen von Verhalten aufgrund externer Hardware
#define extCrystal          0       //Wenn ein externer Quarz verbaut ist

//Die folgenden Definitionen zeigen die Stelle im BYTE Array wo die Einstellungen gespeichert sind:
//Man kann direkt auf das ARRAY per Funk zugreifen die Werte sind immer dezimal:
//"w_5":"3" -> Zum Schreiben einer 3 an Stelle 5
//"r_5":"3" -> Zum Lesen der Stelle 5 (die "3" wird gebraucht allerdings ignoriert)
//"p_5":"1" -> Zum setzen und ruecksetzen des Ports 5 (funktion_pin..)
//"d_5":"ABC123" -> Zum schreiben eines Strings auf das Display. 5 ist die Zeilen Nummer

//Bytes des Config Arrays
//Anordnung wie auf der Leiterplatte
#define  funktion_pin1          0
#define  funktion_pin0          1
#define  funktion_pin19         2
#define  funktion_pin18         3
#define  funktion_pin9          4
#define  funktion_pin16         5
#define  funktion_pin17         6

#define  math_analog2           10
#define  math_analog3           11
#define  math_analog4           12
#define  math_analog5           13
#define  digitalSensors         15
#define  digitalOut             16

#define  chipSetup              19
#define  nodeControll           20
#define  sleepTimeMulti         21    //sleepTime Multiplikator
#define  sleepTime              22    //Sleep time in Sekunden * 8 sec, Wenn der Timer gesetzt ist wird auch die Batteriespannung gemessen
#define  watchdogTimeout        23    //watchdog in * 8 sec, Zeit bis zum abfallen des Watchdogs und setzen der Pins auf den vorgegebenen Zustand (port). Der Sleep wird gesperrt wenn der Watchdog nicht abgefallen ist.
#define  watchdogDelay          24    //watchdog in * 5 sec, Zeit bis zum nachsten senden eines Watchdogs (Nur wenn sleepTime == 0 konfiguriert ist oder der watchdog durch staendiges Nachtriggern den Sleep blokiert. Ansonsten wird der Watchdog sofort nach dem Aufwachen gesendet wenn watchdogDelay>0)

#define  contrast               26
#define  nodeId                 27    //NodeId dieses Sensors (einzigartig im Netzwerk)
#define  networkId              28    //NetworkID dieses Sensors (alle gleich im Netzwerk)
#define  gatewayId              29    //GatewayID an diese Addresse werden die Daten geschickt und es werden nur Daten von dieser Addresse beruecksichitgt. Sollten Daten von einer Anderen Adresse kommen werden sie an das Gateway weitergeleitet.
#define  sensorDelay            30    //Messpause der Sensoren in * 10 sec (Nur wenn sleepTime == 0 konfiguriert ist oder der watchdog durch staendiges Nachtriggern den Sleep blokiert)
#define  firstEEPromInit        31    //internal
#define  oscCalError            32    //internal

//In den folgenden Definitionen ist das PinMapping beschrieben: fur digtal IOs config[0]:config[9], fur analog Is config[10]:config[14].
#define usedDio                 7       //Anzahl der verwendetetn DIOs
#define usedAnalog              4       //Anzahl der verwendetetn analog In
const uint8_t pinMapping[15]{1,0,19,18,9,16,17,0,0,0,2,3,4,5,0};
//der SSPin macht noch Probleme Spi funktioniert dann nicht mehr!
//const uint8_t pinMapping[15]{0,1,17,18,19,9,10,0,0,0,3,4,5,0,0};
unsigned long counter[usedDio];
const uint8_t LedPinMapping[3]{5,6,7};

// Typische Led Farben: 0:Gruen 1:Gelb/Org 2:Rot

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

boolean set_Temperature(uint8_t temperature, boolean setTemp = false);
boolean get_saved_Massages(boolean startup = false);

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void pciDisable(byte pin)
{
    *digitalPinToPCMSK(pin) &= ~bit (digitalPinToPCMSKbit(pin));  // disable pin
}

boolean oscCalibration(void){
    
    //Timer1 laufen lassen
    //TImer2 clock source auf externen pin ohne prescaler
    //beide Timer starten
    //polling auf den overflow
    //den Timer1 stoppen 
    //werte des Timer1 mit dem REF_VAL vergleichen
    //osccal incrementieren oder decrementieren
    //bei einem Timeout die Schleife verlassen und osccal zurueck setzen
    
    //Input Reference Clock on Pin TOSC1          
    #define REF_CLOCK 1000000UL


    #define COUNT_PRE (1 << CS10)
    #define COUNT_PREDIV 1
    
    //Bei extrem langsamer ref Clock und schnellem Internen Takt
    //#define COUNT_PRE (1 << CS11)
    //#define COUNT_PREDIV 8


    #define REF_VAL F_CPU / (REF_CLOCK / 256) / COUNT_PREDIV

    #define REF_MIN REF_VAL - 3

    #define REF_MAX REF_VAL + 3

    cli();
    //setze clock prescaler auf 2. Internal Clock 8MHZ / 2 = 4MHZ. Das ist das maximum bei minimaler Betriebsspannnung von 1.8V.
    clock_prescale_set(1);
    
    uint8_t oldOscal = OSCCAL;
    uint8_t oldTCCR1A = TCCR1A;
    uint8_t oldTCCR1B = TCCR1B;
    uint8_t oldTCCR1C = TCCR1C;
    uint8_t oldTIMSK1 = TIMSK1;
    uint8_t oldTCCR2A = TCCR2A;
    uint8_t oldTCCR2B = TCCR2B;
    uint8_t oldTIMSK2 = TIMSK2;
    uint8_t oldASSR  = ASSR;
    
    //Reset Timer1 und Timer2
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1C = 0;
    TIMSK1 = 0;
    TCCR2A = 0;
    TCCR2B = 0;
    TIMSK2 = 0;
    ASSR = 0;

    uint16_t temp3 = 0;
    uint16_t temp4 = 0;
    
    unsigned char nTimeOut = 1;

    uint8_t oldREG_DIOMAPPING2 = rfm69.readReg(REG_DIOMAPPING2);
    rfm69.writeReg(REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_1); 


    //Timer anhalten, Prescaler Reset
    TIMSK2 = 0;
    GTCCR |= (1 << TSM) | (1 << PSRASY);
    ASSR = (1<<EXCLK);
    ASSR |= (1 << AS2);
    TCCR2B = (1 << CS20);
    TCCR2A = 0;    //Normal operation.  Reset timer on hitting TOP.
    GTCCR &= ~(1 << TSM);//Timer starten
    
    while(!(TIFR2 & (1 << TOV2)));
    //Clear Interrupt Flags
    TIFR2 = (1 << OCF2B) | (1 << OCF2A) | (1 << TOV2);
    TCCR1B = COUNT_PRE;
    
    //OSCAL in die "Mitte" setzen, da wir die Schleife nur jedes zweite Mal auswerten
    OSCCAL = 128;
    
    
    while(nTimeOut){
        //Poll Timer2    
        if(TIFR2 & (1 << TOV2)){
            //Stop Timer1, Prescaler Reset
            GTCCR = (1 << TSM) | (1 << PSRSYNC);
            TIFR2 = (1 << OCF2B) | (1 << OCF2A) | (1 << TOV2);
            unsigned int nTcnt1 = TCNT1;
            if(!(nTimeOut & (1 << 0))){
                if (!temp3){
                    temp3 = nTcnt1;
                }
                if((nTcnt1 >= REF_MIN) && (nTcnt1 <= REF_MAX)){
                    temp4 = nTcnt1;
                    break;
                }else{
                    if(nTcnt1 < REF_VAL){
                        OSCCAL++;
                    }
                    if(nTcnt1 > REF_VAL){
                        OSCCAL--;
                    }
                }
                TCNT1 = 0;
            }else{
                GTCCR = 0;//Start Timer1
            }
            nTimeOut++;
        }
    }
    sei();
    
    rfm69.writeReg(REG_DIOMAPPING2, oldREG_DIOMAPPING2);    

    //Register wiederherstellen
    TCCR1A = oldTCCR1A;
    TCCR1B = oldTCCR1B;
    TCCR1C = oldTCCR1C;
    TIMSK1 = oldTIMSK1;
    TCCR2A = oldTCCR2A;
    TCCR2B = oldTCCR2B;
    TIMSK2 = oldTIMSK2;
    ASSR = oldASSR;
    // Clear Interrupt Flags
    TIFR1 = (1 << ICF1) | (1 << OCF1B) | (1 << OCF1A) | (1 << TOV1);
    TIFR2 = (1 << OCF2B) | (1 << OCF2A) | (1 << TOV2);
    //Start Timer1
    GTCCR = 0;
    
    char temp2[13];
    char wert[5];
    
    if(!nTimeOut)
    {
        //Error, Kalibrierung abgebrochen
        OSCCAL = oldOscal;    //zurück setzen auf den Rest Wert
        //u8x8.drawString(4,7,"Err Cal ");   
        return false;
    }else{
        /*
        strcpy(temp2, "OK " );
        itoa(OSCCAL, wert, 10);
        strncat(temp2, wert, 8);        
        u8x8.drawString(4,7,temp2);
        */
        return true;
    }


}

boolean getJumper(void){

        pinMode(JP_2, INPUT_PULLUP);
        delay(1);
        if (digitalRead(JP_2)){
            return false;
        }else{
            return true;
        }
}

void initVariables(void)
{
    //Standard config laden
    //Alle Pins auf Eingang
    //keine Sensoren aktiv
    //es kann bei der ersten Inbetriebnahme zu Problemen (haengt beim Sensor lesen) kommen wenn diese Variable nicht auf 255 steht
    
    if ((eeprom_read_byte(&eeConfig[0]) == 255) || (eeprom_read_byte(&eeConfig[firstEEPromInit]) != 0xAF) || getJumper()){
    //if (1){
        for (uint8_t i = 0; i < configSizeAll - 5; i++){
            eeprom_write_byte(&eeConfig[i], 0);
        }
        eeprom_write_byte(&eeConfig[contrast], 255);        
        eeprom_write_byte(&eeConfig[nodeId], DEFAULTNODEID);
        eeprom_write_byte(&eeConfig[networkId], DEFAULTNETWORKID);
        eeprom_write_byte(&eeConfig[gatewayId], DEFAULTGATEWAYID);
        eeprom_write_byte(&eeConfig[sensorDelay], 2);
        eeprom_write_byte(&eeConfig[firstEEPromInit], 0xAF);
        eeprom_write_byte(&eeConfig[oscCalError], 0);
        eeprom_write_block(DEFAULTENCRYPTKEY,  &eeEncryptKey, 16+1);
    }
    
    for (uint8_t i = 0; i < configSizeAll; i++){
        config[i] = eeprom_read_byte(&eeConfig[i]);
    }
    
    //reset Pin Counters
    for (uint8_t i = 0; i < usedDio; i++){
        counter[i] = 0;
    }
    
    SensorPeriod = config[sensorDelay] * timemultiplierSensor;
    WatchdogPeriod = config[watchdogDelay] * timemultiplierWD;
    WdPinTimeout = config[watchdogTimeout] * timemultiplierWD;
    WdTrigTimeStamp = millis() - WdPinTimeout;
    //Wir wollen den SLeep erst frei geben wenn der Watchdog abgelaufen ist
    if (WdPinTimeout > 0){
        SleepAlowed = false;
    }else{
        SleepAlowed = true;
    }
    
    // Wenn die node neu ist wird sie vom Gateway nicht empfangen
    if ((config[networkId] == DEFAULTNETWORKID) && (config[nodeId] == DEFAULTNODEID)){
        SendAlowed = false;
    }
    
}

void check_improveConfig(void){
        
        
            
}

void setupPins(void)
{
    //starte ADC for readPlant oder readLDR oder readRain
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
        if (config[i] & (1 << intRise)){	
            pciSetup(pinMapping[i]);
        }
    }	
    
    //Der supplyPin ist zum versorgen der Sensoren gedacht
    if (config[nodeControll] & (1<<sensorPower)){
        digitalWrite(supplyPin, HIGH);
    }    

    if (config[digitalSensors] & (1<<readHC05)){
        pinMode(HC05pingPin, INPUT);
        pinMode(HC05trigPin, OUTPUT);
    }
            
}

void setupDefaultPins(void){
    // Onboard LEDs
    pinMode(LedPinMapping[0], OUTPUT);
    pinMode(LedPinMapping[1], OUTPUT);
    pinMode(LedPinMapping[2], OUTPUT);
    
    //reset Pin Rfm
    pinMode(ResetRfmPin, OUTPUT);
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

    setupDefaultPins();
    //reset_wdPins(); 
    
    digitalWrite(LedPinMapping[2], HIGH);

    //RFM69-------------------------------------------
    //Reset
    digitalWrite(ResetRfmPin, HIGH);
    delay(5);
    digitalWrite(ResetRfmPin, LOW);
    delay(10);
    //Init RFM69

    if (!rfm69.initialize(FREQUENCY, config[nodeId], config[networkId])){
        for (uint8_t i = 0; i<5; i++){
            digitalWrite(LedPinMapping[2], LOW);
            delay(100);
            digitalWrite(LedPinMapping[2], HIGH);
            delay(100);
        }
    }

    //rfm69.initialize(FREQUENCY,NODEID,NETWORKID);
    #ifdef IS_RFM69HW
        rfm69.setHighPower(); 
    #endif
    rfm69.setPowerLevel(RFM_POWER_LEVEL);
    char temp[16+1];
    eeprom_read_block(temp, eeEncryptKey, 16+1);
    rfm69.encrypt(temp);
    //rfm69.encrypt(DEFAULTENCRYPTKEY);

    rfm69.receiveDone();		//goto Rx Mode
  
    //--------------------------------------

    //Spezial Pin Einstellungen laden
    setupPins();
    
    //ssd1306 Display
    if ((config[digitalOut] & (1<<ssd1306_128x64)) || (config[digitalOut] & (1<<ssd1306_64x48))){
        //u8x8.setI2CAddress(0x3c);
        //u8x8.setI2CAddress(0x3d);
        u8x8.begin();
        u8x8.setPowerSave(0);
        u8x8.setContrast(config[contrast]);
        u8x8.setFont(u8x8_font_pxplusibmcga_r);
        char temp[10] = "Id   ";
        char wert[4];
        itoa(config[nodeId], wert, 10);
        strncat(temp, wert, 3);
        u8x8.drawString(4,2,temp);
        strcpy(temp, "Net  " );
        itoa(config[networkId], wert, 10);
        strncat(temp, wert, 3);        
        u8x8.drawString(4,3,temp);       
    }
    
    uint8_t printOK = 1;
    if (!(config[chipSetup] & (1<<extCrystal))){
        if (!(eeprom_read_byte(&eeConfig[oscCalError]))){
            eeprom_write_byte(&eeConfig[oscCalError], 1);
            wdt_enable(WDTO_8S);
            if (!oscCalibration() && SendAlowed) {
                const char errorString[] = "\"state\":\"oscCal\"";
                rfm69.sendWithRetry(config[gatewayId], errorString, sizeof(errorString));
            }
            eeprom_write_byte(&eeConfig[oscCalError], 0);
            disableWd();
        }else if (SendAlowed){
            printOK = 0;
            const char errorString[] = "\"state\":\"oscCal_skiped\"";
            rfm69.sendWithRetry(config[gatewayId], errorString, sizeof(errorString));
        }
    }else if (SendAlowed){
        printOK = 0;
        const char errorString[] = "\"info\":\"oscCal_skiped\"";
        rfm69.sendWithRetry(config[gatewayId], errorString, sizeof(errorString));
    }
    
    //device UART
    if (config[digitalOut] & (1<<uart)){
        Serial.begin(SERIAL_BAUD);
    }
    
    if (config[digitalSensors] & (1<<readBME)){
        if (!bme.begin() && SendAlowed) {
            printOK = 0;
            const char errorString[] = "\"state\":\"error BME Init\"";
            rfm69.sendWithRetry(config[gatewayId], errorString, sizeof(errorString));
        }
    }
        
    //reset oscal error damit der oscal beim nächten MAl wieder ausgeführt wird
    eeprom_write_byte(&eeConfig[oscCalError], 0);

    if (printOK && SendAlowed){
        const char errorString[] = "\"state\":\"Ok\"";
        rfm69.sendWithRetry(config[gatewayId], errorString, sizeof(errorString));
    }

    digitalWrite(LedPinMapping[2], LOW);
}

 boolean write_buffer_str(char *name, char *wert, boolean strWert = false, boolean deleteBuf = false )
{
    /*
    write_buffer_str("abc","123") -> Es wird ein String '"abc":123' in den Buffer geschrieben
    write_buffer_str("abc","123",false) -> Es wird ein String '"abc":"def"' in den Buffer geschrieben	
    write_buffer_str("","") -> Es werden alle Daten gesendet und der Pointer auf 0 gesetzt
    
    Es wird am Anfang ein { und am Ende ein } gesetzt damit die Nachricht ein gueltiges json Format hat
    Es wird immer ueberprueft ob die maxixmale laenge von 61Bytes (begrenzt vom RFM Modul) ueberschritten wird 
    Es wird immer ueberprueft ob die neue Nachricht + messageOverhead noch Platz hat. Wenn nicht wird erst gesendet
    */
    
    #define lengthMessageOverheadStr 6    //"":"",
    #define lengthMessageOverheadDec 4    //"":,
    #define lengthMessageEndChar     1    //}
    
    static char sendBuffer[RF69_MAX_DATA_LEN + 1];
    static uint8_t sendBufferPointer = 0;
    static uint32_t messageId = 0;
    uint8_t nameLength = strnlen(name, RF69_MAX_DATA_LEN);
    uint8_t wertLength = strlen(wert);
    uint8_t messageOverhead;
    
    if (deleteBuf){
        sendBufferPointer = 0;
    }
    
    if (strWert == true){
        messageOverhead = lengthMessageOverheadStr;
    }else{
        messageOverhead = lengthMessageOverheadDec;
    }
    
    //Wir merken uns wann das Senden das letzte Mal schief ging und reseten ggf die Sperre
    static unsigned long sendTimeOld;
    unsigned long timepassed;
    timepassed = millis() - sendTimeOld;
    
    if (timepassed >= 10000){
      sendTimeOld = millis();
      SendAlowed = true;
    }
    
    //Es wird geprueft ob die Nachricht ueberhaupt in den Buffer passt ansonsten senden wir eine Fehlermeldung
    if ((nameLength + wertLength + messageOverhead + lengthMessageEndChar) <= RF69_MAX_DATA_LEN)
    {
        //Wir pruefen ob die Nachricht zusaetlich in den Buffer passt ansonsten schicken wir zuerst alle Daten weg
        //Wenn die wertLange 0 ist, und Daten, die noch nicht gesendet worden sind vorliegen, dann wollen wir diese senden 
        if (SendAlowed){
            if (((nameLength + wertLength + sendBufferPointer + messageOverhead + lengthMessageEndChar) > RF69_MAX_DATA_LEN) || ((wertLength == 0) && (sendBufferPointer > 0))){
                sendBuffer[sendBufferPointer] = '\}';
                if (!rfm69.sendWithRetry(config[gatewayId], sendBuffer, sendBufferPointer + lengthMessageEndChar)){
                    //Wir konnten nicht senden-> wir warten und probieren es noch einmal
                    delay(config[nodeId]*1.3);
                    if (!rfm69.sendWithRetry(config[gatewayId], sendBuffer, sendBufferPointer + lengthMessageEndChar)){
                        //Ein Fehler ist aufgetreten wir merken uns den Bufferinhalt
                        //radio_Rx_loop(); //RFM->RXMode
                        //Wir verbieten das Senden und merken uns den Zeitstempel damit diese Node nicht sofort wieder senden will und die Frequenz blokiert
                        sendTimeOld = millis();
                        SendAlowed = false;
                        return false;
                    }else{
                        sendBufferPointer = 0;
                    }
                }else{
                    sendBufferPointer = 0;
                }
            }
        // es gib den Fall dass die Funktion mit einer wertlenth == 0 aufgerufen wird in diesem Fall wollen wir nur senden 
        // wenn dies aber nicht klappt (z.B.SendAlowed == 0) dann muessen wir false zurueck geben
        }else if ((wertLength == 0) && (sendBufferPointer > 0) &! SendAlowed){
            return false;
        }
        if ((nameLength + wertLength + sendBufferPointer + messageOverhead + lengthMessageEndChar) < RF69_MAX_DATA_LEN){
        // WIr wollen nur in den Puffer schreiben wenn auch ein Wert uebergeben wurde und der Buffer leer ist
            if (wertLength != 0){
                if (sendBufferPointer > 0){
                    sendBuffer[sendBufferPointer++] = ',';
                }else{
                    sendBuffer[sendBufferPointer++] = '\{';
                    const char idMessage[] = "\"id\":";
                    char strmessageId[10];
                    for (uint8_t loop = 0; idMessage[loop] != '\0'; loop++)
                    {
                        sendBuffer[sendBufferPointer++] = idMessage[loop];
                    }
                    itoa(++messageId, strmessageId, 10);
                    for (uint8_t loop = 0; strmessageId[loop] != '\0'; loop++)
                    {
                        sendBuffer[sendBufferPointer++] = strmessageId[loop];
                    }
                    sendBuffer[sendBufferPointer++] = ',';
                }
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
                sendBuffer[sendBufferPointer] = '\0';
            }
        }else{
          //Wenn der Buffer nicht leer ist und nicht beschrieben wurde geben wir ein false zurück
          return false;
        }
    }else{
        const char errorString[] = "\"err\":\"Message to long\"";
        if (SendAlowed){
            rfm69.sendWithRetry(config[gatewayId], errorString, sizeof(errorString));
        }
        //radio_Rx_loop(); //RFM->RXMode
        return false;
    }
    
    //radio_Rx_loop(); //RFM->RXMode
    return true;
}

void sendInt(char *name, uint8_t wert, boolean sendImmidiatelly = true){
    
    char temp[5];
    itoa(wert, temp, 10);
    write_buffer_str(name, &temp[0]);
    if (sendImmidiatelly){
        write_buffer_str("",""); //sende Daten
    }
}

void sendLong(char *name, unsigned long wert, boolean sendImmidiatelly = true){
    
    char temp[5];
    ltoa(wert, temp, 10);
    write_buffer_str(name, &temp[0]);
    if (sendImmidiatelly){
        write_buffer_str("",""); //sende Daten
    }
}

boolean readMessage(char *message){
    // Message str input: "R_12":"125"
    //Diese folgende Schleife macht aus dem String 3 oder eine vielzahl von 3 NULL terminierte Strings
    //o.g. message ergibt parts[0] == "R" , parts[1] == "12", parts[2] == "125", i == 2
    char parts[8][20];
    char *p_start, *p_end;
    uint8_t i = 0;
    p_start = message;
    while(1) {
        p_end = strchr(p_start, '/"');
        if (p_end) {			//search first " to set pointer
            p_start = p_end;
        }
        p_end = strchr(p_start, '_'); 
        if (p_end) {							//copy "R"
            strncpy(parts[i], p_start+1, p_end-p_start-1);
            parts[i][p_end-p_start-1] = 0;
            i++;
            p_start = p_end + 1;
            p_end = strchr(p_start, '/"'); 
            if (p_end) {						//copy "12"
                strncpy(parts[i], p_start, p_end-p_start);
                parts[i][p_end-p_start] = 0;
                i++;
                p_start = p_end + 1;
                p_end = strchr(p_start, '/"');
                if (p_end) {					//prepare to search next "
                    p_start = p_end + 1;
                    p_end = strchr(p_start, '/"');
                    if (p_end) {				//copy "125"
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
        write_buffer_str("part0", parts[0]);
        write_buffer_str("part1", parts[1]);
        write_buffer_str("part2", parts[2]);
        write_buffer_str("part3", parts[3]);
        write_buffer_str("part4", parts[4]);
        write_buffer_str("part5", parts[5]);
        write_buffer_str("part6", parts[6]);
        write_buffer_str("part7", parts[7]);
        write_buffer_str("part8", parts[8]);
        write_buffer_str("part9", parts[9]);
        write_buffer_str("","");
    */
        
    uint8_t listLen = i;
    boolean resetCPU = false;
    
    //folgende Schleife vergleicht die Strings in dem parts array mit den Steuerbefehlen der Node und uebernimmt die Daten
    for (uint8_t i=0; i<=listLen; i++){
        /*
        write_buffer_str("parse", parts[i], true);
        rfm69.sendWithRetry(config[gatewayId], sendBuffer, sendBufferLength);
        delay(100);
        */
        //zum schreiben einer festen config
        if (strcmp(parts[i] , "w") == 0){
            if (atoi(parts[i + 1]) < configSize){
                uint8_t regNr;
                regNr = atoi(parts[i + 1]);
                eeprom_write_byte(&eeConfig[regNr], atoi(parts[i + 2]));
                //Daten in das Abbild uebernehemn
                config[regNr] = eeprom_read_byte(&eeConfig[regNr]);
                char temp[10] = "w_";
                strncat(temp, parts[i+1],2);
                sendInt(temp, config[regNr]);
                i += 2;
                //Wir wollen in einem sauberen Zustand starten manche Werte koennen ohne Neustart uebernommen werden
                if (!((regNr == 20) || (regNr == 21) || (regNr == 22) || (regNr == 23) || (regNr == 24))){
                    resetCPU = true;
                }
            }
        }
        //zum lesen der Config
        if (strcmp(parts[i] , "r") == 0){
            char temp[10] = "w_";
            strncat(temp, parts[i+1],2);
            sendInt(temp, config[atoi(parts[i + 1])]);
            i += 2;
        }
        //zum lesen der gesamten konfig rAll
        if (strcmp(parts[i] , "rAll") == 0){
            for (uint8_t i = 0; i<configSize; i++){
                if (config[i] != 0){
                    char temp[5] = "w_";
                    char temp2[4];
                    itoa(i, temp2, 10);
                    strncat(temp, temp2, 4);
                    itoa(config[i], temp2, 10);               
                    write_buffer_str(temp, temp2, true);
                }
            }
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
            }else if (strcmp(parts[i+2] , "0") == 0){
                digitalWrite(pinMapping[atoi(parts[i + 1])],LOW);
            }
            //damit der Status zurueck gesendet wird setze das Bit "readInput" fuer den jeweiligen Port 
            i += 2;
        }
        if (strcmp(parts[i] , "led") == 0){
            if (strcmp(parts[i+2] , "blink") == 0){
                digitalWrite(LedPinMapping[atoi(parts[i + 1])], HIGH);
                delay(100);
                digitalWrite(LedPinMapping[atoi(parts[i + 1])], LOW);
            }else if(strcmp(parts[i+2] , "1") == 0){
                digitalWrite(LedPinMapping[atoi(parts[i + 1])], HIGH);
            }else{
                digitalWrite(LedPinMapping[atoi(parts[i + 1])], LOW);
            }
        }
        if (strcmp(parts[i] , "key") == 0){
            eeprom_write_block(parts[i+2],  &eeEncryptKey, 16+1);
            resetCPU = true;
        }
        if (strcmp(parts[i] , "d") == 0){
            if (config[digitalOut] & (1<<ssd1306_64x48)){
                u8x8.drawString(4, (atoi(parts[i + 1])+2), "            ");
                u8x8.drawString(4, (atoi(parts[i + 1])+2), parts[i + 2]);
            }else if (config[digitalOut] & (1<<ssd1306_128x64)){
                u8x8.drawString(0, atoi(parts[i + 1]), "                        ");
                u8x8.drawString(0, atoi(parts[i + 1]), parts[i + 2]);
            }
            i += 2;
        }
        if (strcmp(parts[i] , "t") == 0){
            ThermostatPresent = true;
            if (!(set_Temperature(atoi(parts[i +2])))){
                write_buffer_str("t_0", "Error", true);
            }else{
                write_buffer_str("t_0", "Ok", true);
            }
        }
        //den Watchdog nachtriggern
        if (strcmp(parts[i] , "wd") == 0){
            WdTrigTimeStamp = millis();
            sendLong("info", WdTrigTimeStamp, false);
            wdMsgSend = false;
            SleepAlowed = false;		
            i += 2;
        }
    }
    if (resetCPU){
        // Wenn ein Sleep programmiert ist unsubscriben wir uns und das Gateway soll sich merken dass wir schlafen -> 18
        char temp[1];
        temp[0] = 18;
        if (SendAlowed) {
            rfm69.sendWithRetry(config[gatewayId], temp, sizeof(temp));
        }
        write_buffer_str("state", "restart_Node", true);
        write_buffer_str("","");	
        wdt_enable(WDTO_15MS);
        while(1);
    }
}

boolean radio_Rx_loop(void) {
    //pruefen ob daten vom RFM vorliegen
    if (rfm69.receiveDone())
    {
        uint8_t senderId;
        int16_t rssi;
        uint8_t data[RF69_MAX_DATA_LEN];
        uint8_t dataLen;
        if (config[nodeControll] & (1<<debugLed)){
            digitalWrite(LedPinMapping[0], HIGH);
        }
        
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
        if (config[nodeControll] & (1<<debugLed)){
            digitalWrite(LedPinMapping[0], LOW);
        }
        return true;
    }else{
        rfm69.receiveDone(); //put rfm69 in RX mode
        return false;
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
    if (temp_F == 0.0F){
        bme.begin();
    }
    dtostrf(temp_F, 3, 1, Temp);
    write_buffer_str("Bp", &Temp[0]);
    
    //temp_F = bme.readAltitude(SEALEVELPRESSURE_HPA);	//Hoehe in m
    //dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf);
    //dtostrf(temp_F, 3, 1, Temp);
    //write_buffer_str("Ba", &Temp[0]);
        
    //Nur bei BME280 nicht bei BMP280,
    temp_F = bme.readHumidity();
    //dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf);
    dtostrf(temp_F, 3, 1, Temp);
    write_buffer_str("Bh", &Temp[0]);
}

void read_Dallas(void)
{

    float temp_F;
    static boolean init = false;
    
    if (!init){
        dallas.begin();
        init = true;
        delay(1500);
    }
    
    dallas.requestTemperatures();
    
    //DeviceAddress  DS18B20;
    uint8_t DS18B20[8];

    for (uint8_t i = 0; i < dallas.getDeviceCount(); i++){
        char temp[17] = "Dt";
        temp_F = dallas.getTempCByIndex(i);
        //dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf);
        char wert[6];
        dtostrf(temp_F, 3, 1, wert);
        dallas.getAddress(DS18B20, i);
        //Wir wollen nur 3 Stellen der Addresse senden
        for (uint8_t ii = 0; ii<3; ii++){
            char tempA[5];
            itoa(DS18B20[ii], tempA, 10);      
            strncat(temp,tempA,3); 
            if (ii < 2){
                strncat(temp,".",3);
            }
        }
        write_buffer_str(temp, wert);
        if (i > 100){
            write_buffer_str("info", "count error");
            break;
        }
    }
    
    if (dallas.getDeviceCount() == 0){
        init = false;
        write_buffer_str("state", "error no Dallas");
    }   
}

void read_HC05(void){
    //cm = microseconds / 29 / 2;
    write_buffer_str("funk", "HC05 read",true);
}

uint16_t read_Vcc(boolean sendWert = true){
    
    // Lese 1.1V reference gegen AVcc
    uint16_t result;   
    ADMUX = (1<<REFS0) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1);
    delay(2);
    ADCSRA |= (1<<ADSC);
    while (bit_is_set(ADCSRA,ADSC));
    result = ADCL;
    result |= ADCH<<8;	
    result = 1111100 / result; //1023 * Vbandgap(1100mV) / ADC_Wert (-1.35%)
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
            uint16_t adcValue;
            adcValue = analogRead(pinMapping[i]);
            if (config[i] & (1<<readPlant)){
                
            }else if (config[i] & (1<<readRain)){
                
            }else if (config[i] & (1<<readLDR)){
                // GND----[_10K_]--+--LDR----VDD
                //                 |________ADin
                //Wir muessen nichts berechnen
                
            }else if (config[i] & (1<<readRaw)){
                //Wir muessen nichts berechnen
            }else if (config[i] & (1<<readVolt)){
                uint16_t Vref = read_Vcc(false);
                float voltPerCount = Vref / 1023.0;
                voltPerCount *= adcValue;                
                adcValue = voltPerCount;
            }
            if (config[i] & (1<<multi2)){
                adcValue *= 2;
            }
            if (config[i] & (1<<multi4)){
                adcValue *= 4;
            }
            itoa(adcValue,wert,10);
            write_buffer_str(temp,wert);
        }   
    }
}

boolean set_Temperature(uint8_t temperature, boolean setTemp = false){

    #define delayTimeA 5
    #define delayTimeB 20
    static uint8_t sollTemp = 0;
    static uint8_t istTemp = 0;
    boolean retVal;
    
    if (setTemp){
        if (sollTemp != istTemp){
            pciDisable(signalA);
            pciDisable(signalB);
            digitalWrite(signalA, LOW);
            digitalWrite(signalB, LOW);
            // Wir schalten die encoder sperre aus
            if (config[digitalOut] & (1<<lockThermostate)){
                pinMode(signalB, INPUT);
            }
            delay(5);
            if ((digitalRead(signalA) == LOW) || (digitalRead(signalB) == LOW)){
                return false;
            }else{
                retVal = true;
            }            
            //Wir stellen erst auf OFF
            for ( uint8_t i = 0; i < 60; i++){
                pinMode(signalA, OUTPUT);
                delay(delayTimeA);
                pinMode(signalB, OUTPUT);
                delay(delayTimeB);
                pinMode(signalA, INPUT);
                delay(delayTimeA);
                pinMode(signalB, INPUT);
                delay(delayTimeB);
            }
            
            delay(10);
            
            //Wir stellen die Temperatur ein
            for ( uint8_t i = 0; i < sollTemp; i++){
                pinMode(signalB, OUTPUT);
                delay(delayTimeA);
                pinMode(signalA, OUTPUT);
                delay(delayTimeB);
                pinMode(signalB, INPUT);
                delay(delayTimeA);
                pinMode(signalA, INPUT);
                delay(delayTimeB);
            }
            istTemp = sollTemp;
        }
    }else{
        if (temperature >= 5){
            //5° ist das minimum das eingestellt werden kann. Von OFF zu 5° muessen wir 0.5° drehen
            temperature -= 4;
            temperature = temperature * 2;
            temperature -= 1;        
            //Wenn setTemp nicht gesetzt ist merken wir uns die Temperatur nur
            sollTemp = temperature;
        }else{
            sollTemp = 0;
        }
        if (config[digitalOut] & (1<<lockThermostate)){
            pinMode(signalB, INPUT);
        }
        delay(5);
        //Error, Encoder steht falsch
        if ((digitalRead(signalA) == LOW) || (digitalRead(signalB) == LOW)){
            retVal = false;
        }else{
            retVal = true;
        }
    }

    if (config[digitalOut] & (1<<lockThermostate)){
            pinMode(signalB, OUTPUT);
    }

    if (ThermostatPresent){
        pciSetup(signalA);
        pciSetup(signalB);
    }
    
    return retVal;
}

void pump_Sensor_Voltage(void){
    
    for (uint16_t i = 0; i < 10000; i++){
        digitalWrite(pumpPin, HIGH);
        //delay(1);
        digitalWrite(pumpPin, LOW);
        //delay(1);
    }

}

boolean get_saved_Massages(boolean getBackupMsg = false){

    /*
      Following commands controlls the gateway to get old msg for backuptopic or normal topic. The gateway also remembers if node is reachable.
      17: subscribe on NodeTopic / Node reachable
      18: unsubscribe both topics / Node NOT reachable
      19: subsrcibe on BackupTopic / Node reachable
      20: unsubscribe both topics / Node reachable
    */
    
    if (!SendAlowed){
        return false;
    }
    
    uint8_t wiederholung = 1;
    
    if (getBackupMsg){
        wiederholung = 2;
    }
    
    for (uint8_t i = 0; i < wiederholung; i++){
        char temp[1];
        if(getBackupMsg){
            temp[0] = 19;
        }else{
            temp[0] = 17;
        }
        
        if (SendAlowed) {
            rfm69.sendWithRetry(config[gatewayId], temp, sizeof(temp));
        }
        //Wir pruefen bis der Timeout abgelaufen ist ob noch eine Nachricht kommt
        for (uint16_t i = 0; i <= rxPollTime; i++){
            //reset i damit wartezeit von vorne begonnen wird
            if (radio_Rx_loop()){
                i = 0;
            }
            delay(1);
        }

        // Wir wollen erreichbar sein wenn wir nicht schlafen
        if(getBackupMsg || !(config[sleepTime] || config[sleepTimeMulti])){
            temp[0] = 20;
        }else{
            temp[0] = 18;
        }
        if (SendAlowed) {
            rfm69.sendWithRetry(config[gatewayId], temp, sizeof(temp));
        }
        
        getBackupMsg = false;
    }
    
    return true;
}

void go_sleep(boolean shortSleep = false){
    PCinterrupt = false;
    
    if (config[nodeControll] & (1<<debugLed)){
        digitalWrite(LedPinMapping[1], LOW);
    }
    
    if (!(config[nodeControll] & (1<<sensorPowerSleep))){
        digitalWrite(supplyPin, LOW);
    }
    
    rfm69.sleep();

    if (config[digitalSensors] & (1<<readBME)){
        bme.sleepMode();
    }
    
    //Wir loeschen den Buffer damit keine alten Daten aufgehoben werden
    if (config[nodeControll] & (1<<delOldData)){
        write_buffer_str("","",false,true);
    }
    
    //Wenn ein Display verbaut ist und sensorPowerSleep nicht gesetzt ist
    if ((config[digitalOut] & (1<<ssd1306_128x64)) || (config[digitalOut] & (1<<ssd1306_64x48))){
        if (!(config[nodeControll] & (1<<displayAlwaysOn))){
            u8x8.setPowerSave(1);
        }
    }
    
    //todo BME sleep
    uint8_t oldADMUX = ADMUX;
    ADMUX = 0;
    uint8_t oldADCSRA = ADCSRA;
    ADCSRA = 0;
    uint8_t oldPRR = PRR;
    PRR = 0;
    
    //Wir schalten den Watchdog ein. 8sec und Interrupt damit wir wieder aufwachen
    if (!((config[sleepTime] == 255) && (config[sleepTimeMulti] == 255))){
        wdt_set(WDTO_8S);
    }
    
    uint16_t iInit = 0; 
    
    if (shortSleep == true){
        iInit = config[sleepTime] * config[sleepTimeMulti];
        iInit--;
    }
    
    for (uint16_t i = iInit; i < (config[sleepTime] * config[sleepTimeMulti]); i++){
        //Wenn der Sensor versorgt werden soll und die Spannung erhöht werden soll
        if ((config[nodeControll] & (1<<pumpSensorVoltage)) && (config[nodeControll] & (1<<sensorPowerSleep))){
            digitalWrite(supplyPin, HIGH);
            pump_Sensor_Voltage();
            digitalWrite(supplyPin, LOW);
        }

        set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here	    
        sleep_enable();         // enables the sleep bit in the mcucr register
        
        sei();
        
        sleep_mode();           // here the device is actually put to sleep!!
        
        if (BreakSleep){		//Sleep untebrechen wenn ein Interrupt von einem Eingang kam
            break;
        }else if (PCinterrupt){
            PCinterrupt = false;
            i--;
        }
        
    }
    
    disableWd();
    PRR = oldPRR;
    ADMUX = oldADMUX;
    ADCSRA = oldADCSRA;
    SendAlowed = true;
    
    if (config[digitalSensors] & (1<<readBME)){
        bme.normalMode();
    }

    //Wenn der Sensor während des Sleeps nicht versorgt wurde schalten wir ihn hier wieder ein
    if (config[nodeControll] & (1<<sensorPower)){
        digitalWrite(supplyPin, HIGH);
    }    
    
    //rfm69.receiveDone();	//set RFM to RX
    if ((config[digitalOut] & (1<<ssd1306_128x64)) || (config[digitalOut] & (1<<ssd1306_64x48))){
        u8x8.setPowerSave(0);
    }
    if (config[nodeControll] & (1<<debugLed)){
        digitalWrite(LedPinMapping[1], HIGH);
    }
}

boolean read_inputs(boolean readAll = false){
    static boolean readAllInputs = true;
    readAllInputs |= readAll;
    static uint8_t lastPinState = 0;
    
    //#define bit_write(p,m,c) (c ? bit_set(p,m) : bit_clear(p,m))

    for (uint8_t i = 0; i<usedDio; i++){
        boolean inputState;
        inputState = digitalRead(pinMapping[i]) == HIGH;
        //wenn beide Interrupts und der Input nicht gezählt werden soll, dann wachen wir auch auf. Fuer eine eventuelle Wakup Taste damit das Display was anzeigt
        if ((config[i] & (1<<intFall)) && (config[i] & (1<<intRise)) && (!(config[i] & (1<<count)))) {
            BreakSleep = true;
        }
        if ((inputState != ((lastPinState & (1 << i)) != 0)) || readAllInputs){
            if (config[i] & (1<<readInput)){
                char temp[5] = "p_";
                char tempindex[3];
                char wert[3];
                itoa(i, tempindex, 10);
                strncat(temp, tempindex, 2);
                itoa(inputState,wert,10);
                //Wir wollen uns den aktuellen State nur merken wenn das senden geklappt hat (In der Hoffung dass der Pegel dann noch anliegt)
                if (write_buffer_str(temp, wert, true)){
                    bit_write(lastPinState, i, inputState);
                }
                BreakSleep = true;
            }else{
                bit_write(lastPinState, i, inputState);
            }
            
            //WEnn das Thermostat angeschlossen ist und die Pins im Pinmapping sind dann wollen wir diese Pins nicht umconfigurieren
            if (!(ThermostatPresent && ((pinMapping[i] != signalA) || (pinMapping[i] != signalB)))){
                //setzen oder reseten der Pin change interrupts
                if (inputState){
                    if (config[i] & (1<<intFall)){
                        pciSetup(pinMapping[i]);
                    }else{
                        pciDisable(pinMapping[i]);
                    }
                    if (config[i] & (1<<count)){
                        counter[i]++;
                    }
                }else{
                    if (config[i] & (1<<intRise)){
                        pciSetup(pinMapping[i]);
                    }else{
                        pciDisable(pinMapping[i]);
                    }
                }
            }else{
                //wenn es sich um einen Interrupt vom Thermostat handel wollen wir aufwachen
                BreakSleep = true;
                if (!readAllInputs){
                    ShowLeds = true;
                }
                bit_write(lastPinState, i, inputState);
            }
        }
    }	
    readAllInputs = false;
    return 0;
}

void read_counters(void){
  for ( uint8_t i = 0; i <usedDio; i++){
      if (config[i] & (1<<count)){
          char temp[5] = "c_";
          char tempindex[3];
          char wert[10];
          itoa(i, tempindex, 10);
          strncat(temp, tempindex, 2);
          ltoa(counter[i],wert,10);
          write_buffer_str(temp, wert);
      }
  }
}

void reset_wdPins(void){
    
    for (uint8_t i = 0; i<usedDio; i++){
        if (config[i] & (1<<wdReq)){
            if (config[i] & (1<<port)){
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
    unsigned long timepassed;

    static long sensorTimeOld;
    static long watchdogTimeOld;
    static long pumpTimeOld;
    static boolean sensorenLesen = true;
    static boolean wdSenden = false;
    static boolean gotoldmsg = false;
    
    if (config[nodeControll] & (1<<debugLed)){
        digitalWrite(LedPinMapping[2], LOW);
    }
    
    if (config[nodeControll] & (1<<pumpSensorVoltage)){
        timepassed = millis() - pumpTimeOld;
        if (timepassed > 3000){
            pumpTimeOld = millis();
            pump_Sensor_Voltage();
        }
    }
    
    set_Temperature(0,true);
    
    timepassed = millis() - sensorTimeOld;
    //Wir wollen die Sensoren abfragen wenn die Pausezeit vorbei ist, wenn sensorenLesen gesetzt ist, dann lesen wir sofort
    if ((timepassed > SensorPeriod) || (sensorenLesen == true)){
        //wartezeit bis die Sensoren gelesen werden koennen (DS18b20) sensorenLesen ist nur gesetzt wenn wir aus dem Sleep kommen oder die CPU neu gestartet wurde
        if ((sensorenLesen == true) && (config[digitalSensors] & (1<<readDS18))){ 
            delay(1000);
        }
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
        read_analog();
        read_counters();
        //sende aktuellen Status der Ports
        read_inputs(!InputAlredyRead);
        InputAlredyRead = false;
        sensorenLesen = false;
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
        if (!wdMsgSend){
          write_buffer_str("info","wd expiered");
          wdMsgSend = true;
        }
        reset_wdPins();
        SleepAlowed = true;
    }

    if (!gotoldmsg){
        //WIr holen uns die gespeicherten Nachrichten ab
        gotoldmsg = get_saved_Massages(true);
    }
    
    if((config[sleepTime] > 0) && (config[sleepTimeMulti] > 0) && SleepAlowed){
        
        // WIr holen uns die gespeicherten Nachrichten vom Gateway (Server) ab
        get_saved_Massages();
        
        //Wenn das Thermostate angeschlossen ist und der Encoder falsch steht dann zeigen wir das mit Led3 an dazu muss evtl die Sperre ausgeschaltet werden
        if (config[digitalOut] & (1<<lockThermostate)){
            pinMode(signalB, INPUT);
        }        
        if (ShowLeds && ThermostatPresent){
            delay(10);
            ShowLeds = false;
            if (digitalRead(signalA) == LOW || digitalRead(signalB) == LOW){
                // Wir schalten die rote Led ein
                digitalWrite(LedPinMapping[0], HIGH);
                delay(30);
                digitalWrite(LedPinMapping[0], LOW);
            }else{
                // Wir schalten die gruene Led ein
                digitalWrite(LedPinMapping[2], HIGH);
                delay(30);
                digitalWrite(LedPinMapping[2], LOW);            
            }
        }
        if (config[digitalOut] & (1<<lockThermostate)){
            pinMode(signalB, OUTPUT);
        }        
        //Die Temperatur einstellen (die Funktion braucht etwas länger)
        set_Temperature(0,true);
        //Wenn der Watchdog im letzten Schritt nicht getriggert wurde dann rufen wir den Sleep auf
        if (SleepAlowed){
            if (config[nodeControll] & (1<<debugLed)){
                digitalWrite(LedPinMapping[2], LOW);
            }
            // Wenn das configBit sendAgain gesetzt ist wollen wir nach einer wdPeriode wieder aufwachen und erneut senden
            if (config[nodeControll] & (1<<sendAgain)){
                boolean sendAgainOk;
                sendAgainOk = write_buffer_str("","");
                go_sleep(!sendAgainOk);
                sensorenLesen = sendAgainOk;
                wdSenden = sendAgainOk;
            }else{
                //Zum leeren des Buffers und senden aller Daten vor dem Sleep
                write_buffer_str("","");
                go_sleep();
                sensorenLesen = true;	//Wir wollen, dass die Sensoren sofort gelesen werden
                wdSenden = true;		//Wir wollen, dass der Watchdog sofort gesendet wird
            }
        }
    }

    if((config[sleepTime] == 0) || (config[sleepTimeMulti] == 0)){
        radio_Rx_loop();
    }

    //Zum leeren des Buffers und senden aller Daten
    //write_buffer_str("","");
    if (config[nodeControll] & (1<<debugLed)){
        digitalWrite(LedPinMapping[2], HIGH);
    }
    
}//end loop

ISR (PCINT0_vect)
{
    cli();
    //debounce switch
    delay(2);
    sei();
    sleep_disable();        // first thing after waking from sleep:
    boolean oldSendAlowed = SendAlowed;
    SendAlowed = false;
    read_inputs();
    InputAlredyRead = true;
    SendAlowed = oldSendAlowed;
    PCinterrupt = true;
}

ISR (PCINT1_vect)
{
    cli();
    //debounce switch
    delay(2);
    sei();
    sleep_disable();        // first thing after waking from sleep:
    boolean oldSendAlowed = SendAlowed;
    SendAlowed = false;
    read_inputs();
    InputAlredyRead = true;
    SendAlowed = oldSendAlowed;
    PCinterrupt = true;
}

ISR (PCINT2_vect)
{
    cli();
    //debounce switch
    delay(2);
    sei();
    sleep_disable();        // first thing after waking from sleep:
    boolean oldSendAlowed = SendAlowed;
    SendAlowed = false;
    read_inputs();
    InputAlredyRead = true;
    SendAlowed = oldSendAlowed;
    PCinterrupt = true;
}

ISR (WDT_vect)
{
    sleep_disable();        // first thing after waking from sleep:
}