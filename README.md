# esp32-sense-and-send
An 18m flume in the Flume Lab at UBC can benefit from having better water surface level data during experiments. The flume is already equipped with a Parker linear rail that traverses the length of the flume. This project is to avoid the mess of adding another wire connection to the cart on the linear rail. Another ESP32 is needed for ESP-NOW.

## Components
- ESP32C5 x 2
- ST's SATEL-VL53L7CX Breakout Board (ToF Sensor)
- 2.2kOhm resistors x 2

## SATEL-VL53L7CX
There is a lot of contradicting/confusing information on the internet about reading data from these breakout boards. The breakouts are convenient as they can be ready to go right out of the box. I wasn't able to get any data from the through hole pins, but I did get data after soldering flywires to the 9 pads on the PCB. I used 28 gauge stranded wire for the power and ground pins and 32 gauge for everything else. I tried using 22 gauge stranded wire but ripped off a solder pad. Small diameter wires are preferred.

![Soldering the Flywires](./imgs/IMG_4760.JPG)
     ![Flywires Attached, Good Enough](./imgs/IMG_4763.JPG)

    | VL53L7CX Solder Pad Name | ESP32C5 Connection | 
    | --- | --- |
    | INT | NC |
    | I2C_RST | NC |
    | SDA | GPIO_24 |
    | SCL | GPIO_23 |
    | LPn | NC |
    | AVDD | 3.3V |
    | IOVDD | 3.3V |
    | GND | GND |

## Examples for ESP-IDF
- examples/ranging_basic
    This is a minimal example to read data from the ToF sensor. The SDA and SCL are set to be GPIO_24 and GPIO_23 respectively. 

    
     

## Function

sensor -> mcu -> espnow
