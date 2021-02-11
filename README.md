# RFM69 JSON (Battery)Multisensor
 
This project is a multisensor node based on a Atmega328 and RFM69 radio. The Sensor is a ultraLowPower(0.1uA) device supplied from 1.8V to 3.6V.
 
You can connect the node with the [Gateway](https://github.com/boaschti/MQTT_WLan_RFM69_Gateway) via MQTT to your home automation.
Just download .hex and flash it with avr studio. 
Fuses for use without ext crystal are: low_fuses=0xd2 high_fuses=0xd1 extended_fuses=0xfe (extended_fuses=0xff = BrownOut off = UltraLowPower)
Fuses for use with ext crystal are: low_fuses=0xff high_fuses=0xd1 extended_fuses=0xfe (extended_fuses=0xff = BrownOut off = UltraLowPower). And cut the cooper between RFM and crystal inputs (atmega).

### Board
The board has
- 7 GPIOs including 4 analog inputs
- spi connector
- 3 Leds
- Micro step up via capacitor
- 2 levelshifter or nFet to power sensors ect
- dimensions are 19x22mm
 
The board you can order [here](https://www.itead.cc/) or other PCB Services. You also can send a Email, my price is 0,80€.
 
### Software Features
 
**Supported sensors**
- BME280 BMP280
- DS18b20 (in future)
- HC-SR05 (in future)
- every switch (pir (hc-sr501), touch sensors (TTP223), reed kontakt, hall Sensor (DRV5032FADBZT) ect)
- light sensor (LDR)
- every analog Sernsor (rain sensor, plant sensor, ect)
- S0 energy counter
 
**Supported aktors**
- every aktor which can be used on a DIO
- watchdog can clear/set DOs
- ssd1306 displays
- any electronic thermostat with encoder to set temperature
 
**Software**
- very simple configuration via [Gateway](https://github.com/boaschti/MQTT_WLan_RFM69_Gateway)
- communication encrypted
- free configurable via radio, no flash programming needed, every node has the same software
- interrupts wakes the node
- the DO can be cleared/set by internal watchdog
- counters on each DI
- programmable sleep times
- programmable sensor measure times
- programmable watchdog times
- and more... you can see [here](https://github.com/boaschti/MQTT_WLan_RFM69_Gateway/blob/master/pictures/nodeConfig.jpg)
 
### Home Assistant config
note: use different command_topic (anyChar). The topic from broker to node is rfmOut/networkId/nodeId/anyChar, the topic from node to broker is rfmIn/networkId/nodeId/jsonValue
 
**Temperature, humidity, pressure, battery, rssi, counter(p_0-p_6)**
```
sensor:
  - platform: mqtt
    state_topic: 'rfmIn/146/254/Bt'
    name: 'Temperature'
    unit_of_measurement: '°C'
    value_template: '{{ value_json.Bt }}'
  - platform: mqtt
    state_topic: 'rfmIn/146/254/Bh'
    name: 'Humidity'
    unit_of_measurement: '%'
    value_template: '{{ value_json.Bh }}'
  - platform: mqtt
    state_topic: 'rfmIn/146/254/Bp'
    name: 'Pressure'
    unit_of_measurement: 'hPa'
    value_template: '{{ value_json.Bp }}'
  - platform: mqtt
    state_topic: 'rfmIn/146/254/batt'
    name: 'Battery'
    unit_of_measurement: 'mV'
    value_template: '{{ value_json.batt }}'
  - platform: mqtt
    state_topic: 'rfmIn/146/254/c_4'
    name: 'Counter'
    value_template: '{{ value_json.c_4 }}'
  - platform: mqtt
    state_topic: 'rfmIn/146/254/rxInfo'
    name: 'RSSI'
    value_template: '{{ value_json.rssi }}'
```
 
**Switches (input) p_0-p_6**
```
binary_sensor:
  - platform: mqtt
    state_topic: "rfmIn/146/250/p_5"
    name: "PIR"
    payload_on: "1"
    payload_off: "0" 
    device_class: opening
    value_template: '{{ value_json.p_5 }}'
```
 
**input slider to set temperature of digital termostat**
```
# Define input_slider
input_slider:
  target_temp:
    name: Target Heater Temperature Slider
    min: 5
    max: 30
    step: 1
    unit_of_measurement: step 
    icon: mdi:target
 
automation:
   # This automation script runs when the target temperature slider is moved.
   # It publishes its value to the same MQTT topic it is also subscribed to
   # The message is json format
  - alias: Temp slider moved
    trigger:
      platform: state
      entity_id: input_slider.target_temp
    action:
      service: mqtt.publish
      data_template:
        topic: "rfmOut/146/250/2"
        retain: true
        payload: "{\"t_0\":\"{{ states.input_slider.target_temp.state | int }}\"}"
```
 
**Outputs p_0-p_6**
```
switch:
  - platform: mqtt
    name: "Test Switch"
    state_topic: "rfmIn/146/254/p_4"
    command_topic: "rfmOut/146/254/1"
    payload_on: "{\"p_4\":\"1\"}"
    payload_off: "{\"p_4\":\"0\"}"
    optimistic: false
    retain: true
```
 
**Display**
```
switch:
  - platform: mqtt
    name: "Test Switch"
    command_topic: "rfmOut/146/254/1"
    payload_on: "{\"d_0\":\"Switch ON\"}"
    payload_off: "{\"d_0\":\"Switch OFF\"}"
    optimistic: true
    retain: true
```

**Watchdog**
'''
input_boolean:
  trigger_wd_terrasse:
    name: Watchdog Terrasse
    initial: off
  switch_terrasse_m:
    name: Terrasse m
    initial: off

automation:
  - alias: Trigger Watchdog
    trigger:
      platform: state
      entity_id: input_boolean.trigger_wd_terrasse
      to: "on"
    action:
      - service: mqtt.publish
        data_template:
          topic: "rfmOut/146/116/wd"
          retain: false
          payload: "{\"wd_0\":\"0\"}"
      - service: input_boolean.turn_off
        data: 
          entity_id: input_boolean.trigger_wd_terrasse    
  
  - alias: Trigger Watchdog auto
    trigger:
      platform: time
      seconds: 59
    condition:
      condition: or
      conditions:
        - condition: state
          entity_id: input_boolean.switch_terrasse_vl
          state: "on"
        - condition: state
          entity_id: input_boolean.switch_terrasse_m
          state: "on"
    action:
      service: input_boolean.turn_on
      data: 
        entity_id: input_boolean.trigger_wd_terrasse  
        
  - alias: Start Timer Terrasse m
    trigger:
      platform: state
      entity_id: input_boolean.switch_terrasse_m
      to: "on"
    action:
      service: mqtt.publish
      data_template:
        topic: "rfmOut/146/116/p_5"
        retain: false
        payload: "{\"wd_0\":\"0\", \"p_5\":\"1\"}"

  - alias: Schalte Ventil Terrasse m aus
    trigger:
      platform: state
      entity_id: input_boolean.switch_terrasse_m
      to: "off"
    action:
      service: mqtt.publish
      data_template:
        topic: "rfmOut/146/116/p_5"
        retain: false
        payload: "{\"p_5\":\"0\"}"
'''
### Hardware
![alt text](https://github.com/boaschti/MQTT_Node_RFM69/blob/master/pictures/nodeHardware.jpg)
![alt text](https://github.com/boaschti/MQTT_Node_RFM69/blob/master/pictures/nodeHardware2.jpg)


### Housing

**3AAA_PIR_Node**
![alt text](https://github.com/boaschti/MQTT_Node_RFM69/blob/master/pictures/3AAA_pir_node.jpg)
 
**3AAA_PIR_Node inside, modification PIR (nFet or diode bat46 4148 ect)**
![alt text](https://github.com/boaschti/MQTT_Node_RFM69/blob/master/pictures/3AAA_pir_node.jpg)
 
**2AAA_Node**
![alt text](https://github.com/boaschti/MQTT_Node_RFM69/blob/master/pictures/2AAA_node.jpg)

**2AAA_Node_inside**
![alt text](https://github.com/boaschti/MQTT_Node_RFM69/blob/master/pictures/2AAA_node_inside.jpg)

**Display_housing**
![alt text](https://github.com/boaschti/MQTT_Node_RFM69/blob/master/pictures/dispayHousing.jpg)
 
**Mini_Node_1**
![alt text](https://github.com/boaschti/MQTT_Node_RFM69/blob/master/pictures/node1housing.jpg)
 
**Mini_Node_2 use the shown BME/BMP Sensor!**
![alt text](https://github.com/boaschti/MQTT_Node_RFM69/blob/master/pictures/node2Housing.jpg)
 
**Modification_Display**
![alt text](https://github.com/boaschti/MQTT_Node_RFM69/blob/master/pictures/reset_Pin_Display.jpg)
![alt text](https://github.com/boaschti/MQTT_Node_RFM69/blob/master/pictures/Display_resistors.jpg)

