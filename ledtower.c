#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <sys/time.h>
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

int fadeAmount = 5; // will make this adjustable with an MQTT command

// -----------------------------------------------------------------------------
// Linux stuff to test the code from the command line
char arr[256] = { 0 };          // fill with NULL
int idx = 0;
uint64_t ticks;

struct termios orig_termios;

void disableRawMode() {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

// https://viewsourcecode.org/snaptoken/kilo/02.enteringRawMode.html
void enableRawMode() {
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(disableRawMode);     // Restores terminal settings on exit
    struct termios raw = orig_termios;
    //raw.c_lflag  &= ~(ECHO);  // We want echo in testing
    // The next 3 settings keep the read from waiting (eats up the CPU though)
    raw.c_lflag    &= ~(ICANON);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;

    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

// This would be replaced with the MQTT processing
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

// -----------------------------------------------------------------------------
// __linux__, __x86_64__
// Dummy Arduino stuff

void analogWrite(int pin, int value) {
}

void
digitalWrite(int pin, int state) {
}

void pinMode(int pin, int value) {
}

//time_t LL = uint64_t
uint64_t
millis() {
    // unsigned long (32 bits)
    // time_t is a signed 32-bit integer (Unix and Arduino, I think)
    // time_t is a signed 64-bit integer (update Linux)
    struct timeval te;
    gettimeofday(&te, NULL); // get current time
    uint64_t milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // calculate milliseconds
    return((uint64_t) milliseconds);
}

#define OUTPUT  1

// -----------------------------------------------------------------------------

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
LEDs
r - red
y - yellow
g - green
b - blue
p - piezo
* - all (just LEDs???)

Commands
on
off
fflash
sflash
ffade
sfade
buzz

Adjustments
W
X
Y
Z
B

*/

// -----------------------------------------------------------------------------

//
// map(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax)
uint32_t
map(uint32_t value, uint32_t inMin, uint32_t inMax, uint32_t outMin, uint32_t outMax) {
    return ((((value - inMin) * (outMax - outMin))/(inMax - inMin)) + outMin);
}

// map() reduced
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

void
ledReset(LED *led, void *func, int state) {
    led->func  = func;
    led->value = 0;
    led->state = state;
}

// This would be modified for the MQTT processing
void
processLine(char *line) {
    char *ledPtr = line;
    char *cmdPtr;

    // Format: <what> <action>
    // ex: r on (red led on)
    if(!*ledPtr) {
        printf("%lu# Cmd line: %s\n", millis(), line);
        return ;
    }

    // strchr function searches within the string pointed to by s for the character c. It returns a pointer to it.
    cmdPtr = strchr((const char *) line, 0x20);
    *cmdPtr = '\0';       // replace the 0x20 (space) with 0x00 (null)
    ++cmdPtr;             // addvance to the first character in the action
    printf("%lu# Cmd line: %s_+_%s\n", millis(), ledPtr, cmdPtr);

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
        // @FIXME: Handle all
    } else {
        for(int i = 0; i < 4; i++) {
            char c = *ledPtr;
            printf("%lu# %s (%c)\nB: %c (%c)\n", millis(), list[i].nom, *list[i].nom, *ledPtr, c);
            // Find the LED
            if(*list[i].nom == *ledPtr) {
                printf("A: %c\n", *ledPtr);
                //
                if(strcmp("on", cmdPtr) == 0) {
                    list[i].func = &on ;
                    break;
                } else if(strcmp("off", cmdPtr) == 0) {
                    ledReset(&list[i], &off, 0);
                    break;
                } else if(strcmp("sfade", cmdPtr) == 0) {
                    list[i].func  = &sfade ;
                    list[i].state = 5;
                    break;
                } else if(strcmp("ffade", cmdPtr) == 0) {
                    list[i].func  = &ffade ;
                    list[i].state = 10;
                    break;
                } else if(strcmp("sflash", cmdPtr) == 0) {
                    list[i].func = &sflash ;
                    break;
                } else if(strcmp("fflash", cmdPtr) == 0) {
                    list[i].func = &fflash ;
                    list[i].state = 0;
                    list[i].ticks = 0;
                    break;
                } else if(strcmp("exit", cmdPtr) == 0) {
                    //list[i].func = &off ;
                    exit(0);
                }
            }
            // Nope didn't find it
            printf("Nope\n");
        }
    }
}

// -[ commands ]----------------------------------------------------------------
void on(LED *led) {
    // Simple just turn it on
    printf("%lu# Function on(%s)\n", millis(), led->nom);
    analogWrite(led->pin, 255);
}

// Off is the default state
void off(LED *led) {
    // Simple just turn it off
    printf("%lu# Function off(%s)\n", millis(), led->nom);
    analogWrite(led->pin, 0);
    led->value = 0;
    led->state = 0;

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
    **
    **   #define FADE_PEDIOD  3000 // fade time is 3 seconds (this is hard coded, 
    **
    **   unsigned long fadeStartTime; // This needs to be in the LED object
    **
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
    **
    ** LED object
    **
    **   typedef struct _led {
    **       char nom[10];                   // LED name Buzzer\0
    **       int  pin;                       // Pin assignment
    **       void (*func)(struct _led *led); // Function to call
    **       int  state;                     // On/Off for Flash, Inc/Dev for Fade
    **       int  value;                     // the last value for fade
    **       uint64_t ticks;                 // Ticks from the start of the current operation (up/down on/off)
    **   } LED;
    **
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
    int      fadeAmount = led->state;

    printf("%lu# Function sfade(%s, %d, %d)\n", millis(), led->nom, led->value, fadeAmount);

    if(now >= nextDelta) {
        analogWrite(pin, brightness);
        //nextDelta += deltaS(brightness);
        led->ticks += deltaS(brightness);
    }

    // change the brightness for next time through the loop:
    led->value = brightness + fadeAmount;

    // reverse the direction of the fading at the ends of the fade:
    if(led->value <= 0 || led->value >= 255) {
        led->state = -fadeAmount;
    } else {
        led->state = fadeAmount;
    }
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

// -----------------------------------------------------------------------------

/*
//
//
// This ESP32 code is created by esp32io.com
//
// This ESP32 code is released in the public domain
//
// For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-led-blink-without-delay
//

#define LED_PIN     18 // ESP32 pin GPIO18 connected to LED
#define BUTTON_PIN  16 // ESP32 pin GPIO16 connected to button

#define BLINK_INTERVAL 1000  // interval at which to blink LED (milliseconds)

// Variables will change:
int ledState = LOW;   // ledState used to set the LED

int previousButtonState = LOW; // will store last time button was updated

unsigned long previousMillis = 0;   // will store last time LED was updated

void setup() {
  Serial.begin(9600);

  // set the digital pin as output:
  pinMode(LED_PIN, OUTPUT);

  // set the digital pin as an input:
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
  // check to see if it's time to blink the LED; that is, if the difference
  // between the current time and last time you blinked the LED is bigger than
  // the interval at which you want to blink the LED.
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= BLINK_INTERVAL) {
    // if the LED is off turn it on and vice-versa:
    ledState = (ledState == LOW) ? HIGH : LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(LED_PIN, ledState);

    // save the last time you blinked the LED
    previousMillis = currentMillis;
  }

  // check button state's change
  int currentButtonState = digitalRead(BUTTON_PIN);

  if (currentButtonState != previousButtonState) {
    // print out the state of the button:
    Serial.println(currentButtonState);

    // save the last state of button
    previousButtonState = currentButtonState;
  }

  // DO OTHER WORKS HERE
}
*/

#define BLINK_INTERVAL_S 1000  // interval at which to blink LED (milliseconds)
#define BLINK_INTERVAL_F 500   // interval at which to blink LED (milliseconds)

#define HIGH 255
#define LOW  0

void flash(LED *led, int rate) {
    int      pin        = led->pin;
    //nt8_t  brightness = led->value;
    uint64_t lastDelta  = led->ticks;
    int      state      = led->state;

    // check to see if it's time to blink the LED; that is, if the difference
    // between the current time and last time you blinked the LED is bigger than
    // the interval at which you want to blink the LED.
    //unsigned long currentMillis = millis();
    uint64_t now        = millis();

    if (now - lastDelta >= rate) {
        // if the LED is off turn it on and vice-versa:
        state = (state == LOW) ? HIGH : LOW;

        printf("%lu# State now = %d\n", millis(), state);
        // set the LED with the ledState of the variable:
        analogWrite(pin, state);

        // save the last time you blinked the LED
        led->ticks = now;
        led->state = state;
    }

    printf("%lu - %lu = %lu# %s State now = %d\n", millis(), lastDelta, now - lastDelta, led->nom, state);
}

void sflash(LED *led) {
    flash(led, BLINK_INTERVAL_S);
    printf("%lu# Function sflash(%s - %d)\n", millis(), led->nom, led->state);
}

void fflash(LED *led) {
    flash(led, BLINK_INTERVAL_F);
    printf("%lu# Function fflash(%s - %d)\n", millis(), led->nom, led->state);
}

// -----------------------------------------------------------------------------
// Print to stdout the state and value of each LED
void lprintf() {
    for(int i = 0; i < 4; i++) {
        LED *led = &list[i];
        printf("%lu# %-7s: %d (%d)\n", millis(), led->nom, led->state, led->value);
    }
}

// This is code used in the main loop to determine how fast the loop runs
// put the toggle pin on a scope and observer the frequency
//  - is it smooth?
//  - what is the frequency?
//  - 1/frequency will give you the time
int
toggle(int pin, int state) {
    state = state ? 0:1; // Toggle from on to off
    digitalWrite(pin, state);
    return(state);
}

int debugState = 0;

void
setup() {

    enableRawMode();

    pinMode(GPIORED,    OUTPUT); // red
    pinMode(GPIOYELLOW, OUTPUT); // yellow
    pinMode(GPIOGREEN,  OUTPUT); // green
    pinMode(GPIOBLUE,   OUTPUT); // blue
    pinMode(GPIOBUZZ,   OUTPUT); // piezo

    pinMode(DEBUGPIN, OUTPUT);
}

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

// -----------------------------------------------------------------------------
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
