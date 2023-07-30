#ifndef FUNCTIONS_H
extern uint32_t map(uint32_t value, uint32_t inMin, uint32_t inMax, uint32_t outMin, uint32_t outMax);
extern uint64_t millis();
extern void     processLine(char *line);
extern void     process();

extern void disableRawMode();
extern void enableRawMode();

extern void on(LED *led);
extern void off(LED *led);
extern void sfadeUp(LED *led);
extern void sfadeDn(LED *led);
extern void ffadeUp(LED *led);
extern void ffadeDn(LED *led);
extern void sflashHi(LED *led);
extern void sflashLo(LED *led);
extern void fflashHi(LED *led);
extern void fflashLo(LED *led);

extern void sfade(LED *led);
extern void ffade(LED *led);

extern void sflash(LED *led);
extern void fflash(LED *led);

extern void analogWrite(int pin, int value);

#define FUNCTIONS_H
#endif
