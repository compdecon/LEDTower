# LEDTower
ESP32 controlling a R/Y/G/Bl LED Tower via MQTT

# Dev version

I'm pretty sure this won't compile or work in the Arduino env. But if it does it won't do anything useful.

The goal of this version is to allow me to have LEDs that can be on/off/fflash/sflash/ffade/sfade and buzz. I'll also try to add commands that can change the way the slow and fast flash and fade work. All of this without using interrupts.

This is code I'm working on in my spare time so I it's very confusing right now. There are extra functions that I don't need and some other junk. It should compile unter Linux with ```gcc -Wall -O3 ledtower.c; echo $?``` but the a.out will problaby not give you useful information yet. Right now I'm testing ideas that will allow me to track the 4 LEDs and the buzzer. This is not the way I code at work but it is the way I code when I get a spare 5 minutes.
