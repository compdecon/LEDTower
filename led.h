#ifndef LED_H

//extern struct _led;
extern void off();

//typedef void (*Func)(int a , int b );

typedef struct _led {
    char nom[10];                   // LED name Buzzer\0
    int  pin;                       // Pin assignment
    void (*func)(struct _led *led); // Function to call
    int  state;                     // On/Off for Flash, Inc/Dev for Fade
    int  value;                     // the last value for fade
    uint64_t ticks;                 // Ticks from the start of the current operation (up/down on/off)
} LED;

LED list[] = {
    { "red",    1, &off, 0, 0, 0 },
    { "yellow", 1, &off, 0, 0, 0 },
    { "green",  1, &off, 0, 0, 0 },
    { "blue",   1, &off, 0, 0, 0 },
    { "piozo",  1, &off, 0, 0, 0 }
};

/*
char *cmds[] = {
    "r on",
    "r off",
    "y on",
    "y off",
    "g on",
    "g off",
    "b on",
    "b off",
    "* on",
    "* off",
    "",
    "* off"
};
*/

#define LED_H
#endif
