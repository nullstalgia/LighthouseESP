# LighthouseESP

Control SteamVR Lighthouses/Base Stations with an ESP32 via BLE!

## Inspired by [ShayBox's Lighthouse](https://github.com/ShayBox/Lighthouse)

### Should be compatible with both V1 (HTC) and V2.0 SteamVR Base Stations/Lighthouses!

Required Libraries:

[JC_Button](https://github.com/JChristensen/JC_Button) (wrote with v2.1.2, quick button debouncing and state machining)

[NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) (Wrote with v1.2.0, MUCH faster than stock BLE GATT)

If you have a V1 Base Station, you *need* to supply the ID from the back of the device in the sketch (src/main.cpp).

If you have V2, it should just work with all of them, but you can filter it to certain ones inside the sketch if you so wish. 

Connect an LED to a pin to get info on if there was an issue during the command (if an error does happen, just try it again a couple times)

- Just a couple slow-ish blinks: Success

- Many fast blinks: Error, try again

If you're lazy and don't want platformio, just copy the contents of main.cpp into the Arduino IDE after installing the ESP32 platform (wrote with 1.0.5) and the libraries.
