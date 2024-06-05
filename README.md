My fork of the Digikey Solder Reflow Oven Build here: https://github.com/bytesizedengineering/Solder-Reflow-Oven/

Adapted to use TFT eSPI library--loads of displays supported and much faster. But you need a pretty fast processor with a good amount of RAM. Think ESP32 and above.

But adapted the code to be pretty much BYO-Display, Touch, and Thermocouple Amp.

Set up your display with TFT eSPI (pins and driver chip can be selected in platformio.ini file). Sorry, Arduino IDE friendly code to follow. But if you know what you're doing, for Arduino IDE, you want to set up pins and driver in the User_Setup.h file in the eSPI lib.

My set-up uses the FT6336 touch driver and a MCP9600 Thermocouple Amp (heard these are more reliable than the 31856's, which can throw flags and interrupt operation). But you can easily set code up to use what you have.

For Touch and Thermocouple, see instructions at top of sketch on how to change the #defines to your specific calls. I set it up so you only need to change 2 lines of code. Then set up your device-specific libs as normal (and remove #includes to libs I used and their associated instance calls).

Right now, code is set up for 480x320 displays, but I'll adapt it to work with smaller displays later (Basically it will just be a font fix).