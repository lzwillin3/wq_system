Inital Commit of code to run on Raspberry PI that measures stream depth and pump water into a climate controled trailer that has a basin that is rinsed and then sampled from.

Devices and equipment actively controlled by the Raspberry Pi:
1. Stream Water Pump (120V AC, 2A pump controlled via GPIO 17 and the Digital Loggers IoT Power Relay II)
2. 1" Tank Drain Valve (12V DC 2-wire auto-return motorized ball valve controlled via GPIO 27 and a 12V opto-isolated relay)
3. Teledyne ISCO 6712 Autosampler #1 (isco_01) (Triggered via RS-232 serial TAKE commands every 90 minutes when stream water level is > 0.01 ft)
4. Teledyne ISCO 6712 Autosampler #2 (isco_02) (Triggered via RS-232 serial TAKE commands when both stream water level and turbidity exceed storm thresholds, with a 15-minute cooldown)
5. Trailer Space Heater (120V AC heater controlled via GPIO 22 and a dedicated IoT Power Relay II when trailer temperature drops below 65 °F)
6. Trailer Air Conditioner (Controlled via GPIO 23 when trailer temperature rises above 80 °F, with a 5-minute compressor short-cycle delay)
7. Decagon 5TM / METER TEROS 11 Sensor #1 (5tm_01) (Measures stream water temperature via SDI-12)
8. Decagon 5TM / METER TEROS 11 Sensor #2 (5tm_02) (Measures trailer interior air temperature via SDI-12)
9. YSI Amazon Bubbler (ysi_bubbler) (Measures stream water level/stage directly in the stream via SDI-12)
10. Campbell Scientific CS451 Water Level Transducer (cs451_tank) (Measures water level and temperature inside the pumped sampling tank via SDI-12)
11. Hach Solitax sc Sensor (hach_solitax) (Measures turbidity and suspended solids in the tank via the Hach SC4500)
12. Hach Nitratax / NT3100sc Sensor (hach_nitratax) (Measures nitrate concentration in the tank via the Hach SC4500)
13. Hach SC4500 Digital Controller (Powers the Solitax and Nitratax probes and serves their measurements to the Raspberry Pi over Modbus TCP)
