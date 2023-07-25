#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <stdint.h> // uintXX_t

// I need to get the correct pins for now just dummy them up
#define DEBUGPIN   0
#define GPIORED    4
#define GPIOYELLOW 5
#define GPIOGREEN  6
#define GPIOBLUE   7
#define GPIOBUZZ   8

// Need to clean this up
#include "led.h"
#include "functions.h"

/*
I asked ChatGPT 4, Bard and Hugging Face to create this. This is mostly from
ChatGPT 4. I've made a few minor changes but will make more.

As the end user I want the R/Y/G/B LED Tower to be used to indicate information about on going events. I want the LEDS to be able to slow flash on and off or fast flash on and off or fade from off to on back to on or 

- ESP32
  - GPIO 4 - Red
  - GPIO 5 - Yellow
  - GPIO 6 - Green
  - GPIO 7 - Blue
  - GPIO 8 - Buzzer
- Arduino enviroment
- MQTT
  - accept command strings from the device/cmd topic
    - coomands are in the format of "color cmd"
      - where command is:
        - on
        - off
        - fflash (fast flash)
        - sflash (slow flash)
      - where color is
        - red
        - green
        - yellow
        - blue
      - addition commands:
        - all off
        - all on
      - commands can be issued asynchronously
  - states can be monitored on device/state
- Flash fast
- Flash slow
- Fade on and off

GPIO -> MOSFET -> LED ->>> GND

Fast Blink 1.0Hz ( 500 mS on,  500 mS off)
Slow Blink 0.5Hz (1000 mS on, 1000 mS off)

Fast Fade  500 ms Up,  500 mS down (10 steps of +/-5 change)
Slow Fade 1000 mS Up, 1000 mS down (20 steps of +/-5 change)

0 -w- 20 -w- 40 ... -w- 1000
map()
0 -w- 5 -w- 10 ... -w- 255

  now = millis();
  if(now >= nextDelta) {
    analogWrite(pin, brightness);
    nextDelta += delta(brightness);
  }
  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
  }

*/
/*
Simlation of the loop processing for commands to the ESP32/LED Tower

r
y
g
b
*
on
off
fflash
sflash
ffade
sfade
*/

char arr[256] = { 0 };

struct termios orig_termios;
int idx = 0;

uint64_t ticks;
int fadeAmount = 5; // will make this adjustable with an MQTT command

// -----------------------------------------------------------------------------

//
// map(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax)
uint32_t
map(uint32_t value, uint32_t inMin, uint32_t inMax, uint32_t outMin, uint32_t outMax) {
    return ((((value - inMin) * (outMax - outMin))/(inMax - inMin)) + outMin);
}

// value, 0, 155, 0, 1000
// ((((value - 0) * (1000 - 0))/(255 - 0)) + 0)
// ((((value) * (1000))/(255))
uint16_t
deltaS(uint8_t value) {
    return(value * 4); // 1000 / 255 ~= 4 (3.9...)
}

uint16_t
deltaF(uint8_t value) {
    return(value * 2); // 500 / 255 ~= 2
}

//time_t
uint64_t
millis() {
    // unsigned long (32 bits)
    // time_t is a signed 32-bit integer (Unix and Arduino, I think)
    // time_t is a signed 64-bit integer (update Linux)
    return((uint64_t) time(NULL));
}

void
processLine(char *line) {
    char *ledPtr = line;
    char *cmdPtr;

    // Format: <what> <action>
    // ex: r on (red led on)
    if(!*ledPtr) {
        printf("Cmd line: %s\n", line);
        return ;
    }
    // strchr function searches within the string pointed to by s for the character c. It returns a pointer to it.
    cmdPtr = strchr((const char *) line, 0x20);
    *cmdPtr = '\0';       // replace the 0x20 (space) with 0x00 (null)
    ++cmdPtr;             // addvance to the first character in the action
    printf("Cmd line: %s_+_%s\n", ledPtr, cmdPtr);

    // Find r, y, g, b, *, B, W, X, Y, Z
    if(isupper(*ledPtr)) {
        printf("Is Upper\n");
        // Special commands to change how long it takes to fade, flash or buzz
        switch(*ledPtr) {
            case 'W':
                // ffade
                break;
            case 'X':
                // sfade
                break;
            case 'Y':
                // fflash
                break;
            case 'Z':
                // sflash
                break;
            case 'B':
                // Buzz
                break;
            // Ignore everything else
        }
    } else if(*ledPtr == '*') {
        printf("Is All\n");
        // Handle all
    } else {
        for(int i = 0; i < 4; i++) {
            char c = *ledPtr;
            printf("%s (%c)\nB: %c (%c)\n", list[i].nom, *list[i].nom, *ledPtr, c);
            // Find the LED
            if(*list[i].nom == *ledPtr) {
                printf("A: %c\n", *ledPtr);
                //
                if(strcmp("on", cmdPtr) == 0) {
                    list[i].func = &on ;
                    break;
                } else if(strcmp("off", cmdPtr) == 0) {
                    list[i].func = &off;
                    break;
                } else if(strcmp("sfade", cmdPtr) == 0) {
                    list[i].func = &sfadeUp ;
                    break;
                } else if(strcmp("ffade", cmdPtr) == 0) {
                    list[i].func = &ffadeUp ;
                    break;
                } else if(strcmp("sflash", cmdPtr) == 0) {
                    list[i].func = &sflashHi ;
                    break;
                } else if(strcmp("fflash", cmdPtr) == 0) {
                    list[i].func = &fflashHi ;
                    break;
                }
            }
            // Nope didn't find it
            printf("Nope\n");
        }
    }
}

void
process() {
    char cmd[257] = { 0 };
    char *line    = cmd;

    ssize_t count;

    line[256] = '\0';

    /*
    If read() is reading from a terminal in non-canonical/raw mode, read will have
    access to keypresses immediately. If you ask read() to get 3 characters it might
    return with anywhere from 0 to 3 characters depending on input timing and how
    the terminal was configured.

    read() will behave differently in the face of signals, returning with less than
    the requested number of characters, or -1 with errno set to EINTR if a signal
    interrupted the read before any characters arrived.

    read() will behave differently if the descriptor has been configured for
    non-blocking I/O. read() will return -1 with errno set to EAGAIN or EWOULDBLOCK
    if no input was immediately available. This applies to sockets.
    */

    // Read a character at a time until we get a newline then 'send' the buffer
    count = read(STDIN_FILENO, &cmd, 1);
    if(count) {
        for(int i = 0; i < count; i++) {
            if(*line == '\n') {
                arr[idx] = '\0';
                idx = 0;
                processLine(arr);
            } else {
                arr[idx++] = *line;
                line++;
            }
        }
    }
}

void disableRawMode() {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

// https://viewsourcecode.org/snaptoken/kilo/02.enteringRawMode.html
void enableRawMode() {
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(disableRawMode);
    struct termios raw = orig_termios;
    //raw.c_lflag  &= ~(ECHO);
    raw.c_lflag    &= ~(ICANON);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;

    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

// -[ commands ]----------------------------------------------------------------
void on(LED *led) {
    // Simple just turn it on
    printf("Function on(%s)\n", led->nom);
}

// Off is the default state
void off(LED *led) {
    // Simple just turn it off
    printf("Function off(%s)\n", led->nom);
}

void analogWrite(int pin, int value) {
}

//
/*
Decisions:
 1. how long is a fade (up and down
*/
void
fade(int pin, int value, int period) {
    uint8_t pwm;

    /*
    ** How to program
    **
    ** Configure an ESP32's pin as a digital output using pinMode()
    **   pinMode(pin, OUTPUT)
    ** Set brightness of the LED by generating a PWM signal with corresponding duty cycle
    ** by using analogWrite()
    **   analogWrite(pin, brightness)
    **
    ** where brightness is a value from 0 - 255
    **
    ** https://esp32io.com/tutorials/esp32-led-fade
    **   // fade-in in loop, and restart after finishing
    **   void loop() {
    **       unsigned long progress = millis() - fadeStartTime;
    **
    **       if (progress <= FADE_PEDIOD) {
    **           long brightness = map(progress, 0, FADE_PEDIOD, 0, 255);
    **           analogWrite(LED_PIN, brightness);
    **       } else {
    **           fadeStartTime = millis(); // restart fade again
    **       }
    **   }
    */

    // period == Delta time (change frequency)
    if (value <= period) {
        // map 0 - FADE_PERIOD (0- 255) to 0 - 255)
        pwm = 255 - map(value, 0, period, 0, 255);
        analogWrite(pin, pwm);
    } else {
        //fadeStartTime = millis(); // restart fade again
    }
}

// -----------------------------------------------------------------------------
// Flash Hi & Lo swap back and forth as each limit is reached
void sfade(LED *led) {
    int      pin        = led->pin;
    uint8_t  brightness = led->value;
    uint64_t nextDelta  = led->ticks;
    uint64_t now        = millis();

    if(now >= nextDelta) {
        analogWrite(pin, brightness);
        nextDelta += deltaS(brightness);
    }

    // change the brightness for next time through the loop:
    brightness = brightness + fadeAmount;

    // reverse the direction of the fading at the ends of the fade:
    if (brightness <= 0 || brightness >= 255) {
        fadeAmount = -fadeAmount;
    }
}

void sfadeUp(LED *led) {
    /*
    ** This is not correct at this point
    **
    **
    */
    printf("Function sfadeUp(%s)\n", led->nom);
    led->func = &sfadeDn;
}

void sfadeDn(LED *led) {
    /*
    ** This is not correct at this point
    **
    **
    */
    printf("Function sfadeDn(%s)\n", led->nom);
    led->func = &sfadeUp;
}

void ffade(LED *led) {
    int      pin        = led->pin;
    uint8_t  brightness = led->value;
    uint64_t nextDelta  = led->ticks;
    uint64_t now        = millis();

    if(now >= nextDelta) {
        analogWrite(pin, brightness);
        led->ticks += deltaF(brightness);
    }

    // change the brightness for next time through the loop:
    brightness = brightness + fadeAmount;

    // reverse the direction of the fading at the ends of the fade:
    if (brightness <= 0 || brightness >= 255) {
        fadeAmount = -fadeAmount;
    }
}

// Flash Hi & Lo swap back and forth as each limit is reached
void ffadeUp(LED *led) {
    /*
    ** This is not correct at this point
    **
    **
    */
    printf("Function ffadeUp(%s)\n", led->nom);
    led->func = &ffadeDn;
}

void ffadeDn(LED *led) {
    /*
    ** This is not correct at this point
    **
    **
    */
    printf("Function ffadeDn(%s)\n", led->nom);
    led->func = &ffadeUp;
}

// -----------------------------------------------------------------------------

// Flash Hi & Lo swap back and forth as each limit is reached
void sflashHi(LED *led) {
    /*
    ** This is not correct at this point
    **
    ** if value > X then switch function to fflashLo
    */
    printf("Function sflashHi(%s)\n", led->nom);
    led->func = &sflashLo;
}

void sflashLo(LED *led) {
    /*
    ** This is not correct at this point
    **
    ** if value > X then switch function to fflashLo
    */
    printf("Function sflashLo(%s)\n", led->nom);
    led->func = &sflashHi ;
}

// Flash Hi & Lo swap back and forth as each limit is reached
void fflashHi(LED *led) {
    /*
    ** This is not correct at this point
    **
    ** if value > X then switch function to fflashLo
    */
    // Fast flash
    /*
    digitalWrite(ledPin, HIGH);
    delay(100); // 100 ms delay for fast flash
    digitalWrite(ledPin, LOW);
    delay(100); // 100 ms delay for fast flash

    ledState[led] = "fflash";  // update state
    */
    printf("Function fflashHi(%s)\n", led->nom);
    led->func = &fflashLo ;
}

void fflashLo(LED *led) {
    /*
    ** This is not correct at this point
    **
    ** if value < N then switch function to fflashHi
    */
    // Fast flash
    /*
    digitalWrite(ledPin, HIGH);
    delay(100); // 100 ms delay for fast flash
    digitalWrite(ledPin, LOW);
    delay(100); // 100 ms delay for fast flash

    ledState[led] = "fflash";  // update state
    */
    printf("Function fflashLo(%s)\n", led->nom);
    led->func = &fflashHi;
}

// https://www.geeksforgeeks.org/how-to-declare-a-pointer-to-a-function/
/*
    // fun_ptr is a pointer to function fun() 
    void (*fun_ptr)(int) = &fun;
  
    // The above line is equivalent of following two
    // void (*fun_ptr)(int);
    // fun_ptr = &fun;
  
    // Invoking fun() using fun_ptr
    (*fun_ptr)(10);
*/

// Print to stdout the state and value of each LED
void lprintf() {
    for(int i = 0; i < 4; i++) {
        LED *led = &list[i];
        printf("%-7s: %d (%d)\n", led->nom, led->state, led->value);
    }
}

void
digitalWrite(int pin, int state) {
}

int
toggle(int pin, int state) {
    state = state ? 0:1; // Toggle from on to off
    digitalWrite(pin, state);
    return(state);
}

int debugState = 0;

void
loop() {
    ticks = millis();

    // get the command
    // process the command
    process();
    // check each LED for current state
    //   if on     on()
    //   if off    off()
    //   if ffade  ffade()
    //   if sfade  sfade()
    //   if fflash fflash()
    //   if sflash sflash()
    //   if buzzer buzzer()
    for(int i = 0; i < 5; i++) {
        // run the function list[i]->func(i)
        list[i].func(&list[i]);
    }

    debugState = toggle(DEBUGPIN, debugState); // put a scope on it to determine the speed of the loop
    //lprintf();
}

// Dummy
void pinMode(int pin, int value) {
}

#define OUTPUT  1

void
setup() {

    enableRawMode();

    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(DEBUGPIN, OUTPUT);
}

int
main(int argc, char **argv) {

    setup();
    
    // Emulate Arduino loop
    while(1) {
        loop();
    }

    exit(0);
}

/*

Notes:

Tower Light - Red Yellow Green Alert Light with Buzzer - 12VDC
https://www.adafruit.com/product/2993
N-channel power MOSFET - 30V / 60A
https://www.adafruit.com/product/355
RGB LED Strips > Usage
https://learn.adafruit.com/rgb-led-strips/usage

How to control an LED strip with RaspberryPi/ESP32 using N-channel MOSFETs?
https://electronics.stackexchange.com/questions/607429/how-to-control-an-led-strip-with-raspberrypi-esp32-using-n-channel-mosfets

https://esp32io.com/tutorials/esp32-led-fade
https://www.arduino.cc/reference/en/language/functions/math/map/
https://deepbluembedded.com/map-function-embedded-c/#:~:text=The%20map%20function%20is%20commonly,certain%20domain%20to%20another%20domain.

//
// This ESP32 code is created by esp32io.com
//
// This ESP32 code is released in the public domain
//
// For more detail (instruction and wiring diagram), visit:
//   https://esp32io.com/tutorials/esp32-led-fade
//

#define LED_PIN  18 // ESP32 pin GIOP18 connected to LED

int brightness = 0;  // how bright the LED is
int fadeAmount = 5;  // how many points to fade the LED by

// the setup routine runs once when you press reset:
void setup() {
  // declare pin GIOP18 to be an output:
  pinMode(LED_PIN, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  // set the brightness of pin GIOP18:
  analogWrite(LED_PIN, brightness);

  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
  }
  // wait for 30 milliseconds to see the dimming effect
  delay(30);
}
0 -w- 5 -w- 10 -w- ... -w- 255
   30mS

.030 * 51 = 1.53 S

About 3 seconds to fade up and fade down

3 seconds is slow
*/


/*
Commands
r on
g fflash
y sflash


o toggle
*/
