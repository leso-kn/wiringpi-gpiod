/*
 *  SPDX-License-Identifier: MIT
 *  Copyright © 2023 Lesosoftware https://github.com/leso-kn.
 *
 *  wiringpi-gpiod - Main library implementation.
 */

#include "wiringPi.h"

#include <malloc.h>
#include <memory.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <gpiod.h>

#include <pthread.h>
#include <sched.h>
#include <sys/time.h>
#include <unistd.h>

//

#define LOG_WARN "[\033[33mwarn\033[0m] "

struct gpiod_chip *chip = NULL;
static uint64_t    epochMilli, epochMicro;

//

const char *piModelNames[]    = {"generic"};
const char *piRevisionNames[] = {"generic"};
const char *piMakerNames[]    = {"generic"};
const int   piMemorySize[]    = {0};

struct wiringPiNodeStruct *wiringPiNodes = NULL;

/*
 *  Export variables for the hardware pointers
 */

volatile unsigned int *_wiringPiGpio        = 0;
volatile unsigned int *_wiringPiPwm         = 0;
volatile unsigned int *_wiringPiClk         = 0;
volatile unsigned int *_wiringPiPads        = 0;
volatile unsigned int *_wiringPiTimer       = 0;
volatile unsigned int *_wiringPiTimerIrqRaw = 0;

int wiringPiFailure(int fatal, const char *message, ...) {
    va_list argp;
    va_start(argp, message);
    vfprintf(stderr, message, argp);
    va_end(argp);

    exit(fatal);
}

/*
 *  Core wiringPi functions
 */

struct wiringPiNodeStruct *wiringPiFindNode(int pin) {
    fprintf(stderr, LOG_WARN
            "wiringPiFindNode() called\n       "
            "wiring-pi-nodes are currently not supported by wiringpi-gpiod\n");
    return malloc(sizeof(struct wiringPiNodeStruct));
};

struct wiringPiNodeStruct *wiringPiNewNode(int pinBase, int numPins) {
    fprintf(stderr, LOG_WARN
            "wiringPiNewNode() called\n       "
            "wiring-pi-nodes are currently not supported by wiringpi-gpiod\n");
    return malloc(sizeof(struct wiringPiNodeStruct));
}

void wiringPiVersion(int *major, int *minor) {
    *major = WIRINGPIGEN_VERSION_MAJOR;
    *minor = WIRINGPIGEN_VERSION_MINOR;
}

int wiringPiSetup(void) {
    chip = gpiod_chip_open("/dev/gpiochip0");

    // Initialize Epoch
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    epochMilli = (uint64_t)ts.tv_sec * (uint64_t)1000 +
                 (uint64_t)(ts.tv_nsec / 1000000L);
    epochMicro = (uint64_t)ts.tv_sec * (uint64_t)1000000 +
                 (uint64_t)(ts.tv_nsec / 1000L);

    return 0;
};

int wiringPiSetupSys(void) { return wiringPiSetup(); };
int wiringPiSetupGpio(void) { return wiringPiSetup(); };
int wiringPiSetupPhys(void) { return wiringPiSetup(); };

void pinModeAlt(int pin, int mode) {
    fprintf(stderr,
            LOG_WARN "pinModeAlt() called\n       "
                     "alternative pin modes are currently not supported by "
                     "wiringpi-gpiod\n");
};

void pinMode(int pin, int mode) {
    struct gpiod_line *line = gpiod_chip_get_line(chip, pin);

    switch (mode) {
    case OUTPUT:
        if (gpiod_line_set_direction_output(line, 1) != 1)
            gpiod_line_request_output(line, "wiringpi-gpiod", 1);
        break;
    case INPUT:
        if (gpiod_line_set_direction_input(line) != 0)
            gpiod_line_request_input(line, "wiringpi-gpiod");
        break;
    }
};

void pullUpDnControl(int pin, int pud) {
    struct gpiod_line *line = gpiod_chip_get_line(chip, pin);

    switch (pud) {
    case PUD_OFF:
        gpiod_line_set_flags(line, GPIOD_LINE_REQUEST_FLAG_BIAS_DISABLE);
        break;
    case PUD_DOWN:
        gpiod_line_set_flags(line, GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN);
        break;
    case PUD_UP:
        gpiod_line_set_flags(line, GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP);
        break;
    }
};

int digitalRead(int pin) {
    struct gpiod_line *line = gpiod_chip_get_line(chip, pin);
    return gpiod_line_get_value(line);
};

void digitalWrite(int pin, int value) {
    struct gpiod_line *line = gpiod_chip_get_line(chip, pin);

    if (gpiod_line_direction(line) == GPIOD_LINE_DIRECTION_INPUT)
        gpiod_line_set_flags(line, value
                                       ? GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP
                                       : GPIOD_LINE_REQUEST_FLAG_BIAS_DISABLE);
    else
        gpiod_line_set_value(line, value);
};

void pwmWrite(int pin, int value) {
    fprintf(stderr,
            LOG_WARN "pwmWrite() called\n       "
                     "pwm is currently not supported by wiringpi-gpiod\n");
}

int analogRead(int pin) {
    fprintf(stderr,
            LOG_WARN "analogRead() called\n       "
                     "analog is currently not supported by wiringpi-gpiod\n");
};

void analogWrite(int pin, int value) {
    fprintf(stderr,
            LOG_WARN "analogWrite() called\n       "
                     "analog is currently not supported by wiringpi-gpiod\n");
};

/*
 *  PiFace specifics
 *  (Deprecated)
 */

// @deprecated PiFace functionality is deprecated
int wiringPiSetupPiFace(void) {
    fprintf(stderr, LOG_WARN
            "wiringPiSetupPiFace() called\n       "
            "Pi-faces are currently not supported by wiringpi-gpiod\n");
};

/* Don't use this - for gpio program only
 * @deprecated PiFace functionality is deprecated
 */
int wiringPiSetupPiFaceForGpioProg(void) {
    fprintf(stderr, LOG_WARN
            "wiringPiSetupPiFaceForGpioProg() called\n       "
            "Pi-faces are currently not supported by wiringpi-gpiod\n");
}

/*
 *  On-Board Raspberry Pi hardware specific stuff
 */

int piGpioLayout(void) { return gpiod_chip_num_lines(chip) > 26 ? 2 : 1; }

// @deprecated Use piBoardId() instead
int piBoardRev(void) {
    int rev = 0;
    piBoardId(NULL, &rev, NULL, NULL, NULL);
    return rev;
}

void piBoardId(int *model, int *rev, int *mem, int *maker, int *overVolted) {
    switch (piGpioLayout()) {
    case 1:
        *model      = PI_MODEL_A;
        *rev        = PI_VERSION_1;
        *mem        = 0;
        *maker      = PI_MAKER_UNKNOWN;
        *overVolted = 0;
        break;
    case 2:
        *model      = PI_MODEL_3BP;
        *rev        = PI_VERSION_2;
        *mem        = 0;
        *maker      = PI_MAKER_UNKNOWN;
        *overVolted = 0;
        break;
    }
}

int wpiPinToGpio(int wpiPin) {
    fprintf(stderr, LOG_WARN
            "wpiPinToGpio() called\n       "
            "not converting value, wiringpi-gpiod uses physical pin numbers\n");
    return wpiPin;
}

int physPinToGpio(int physPin) {
    fprintf(stderr, LOG_WARN
            "physPinToGpio() called\n       "
            "not converting value, wiringpi-gpiod uses physical pin numbers\n");
    return physPin;
}

void setPadDrive(int group, int value) {}

int getAlt(int pin) {
    fprintf(stderr,
            LOG_WARN "getAlt() called\n       "
                     "alternative pin modes are currently not supported by "
                     "wiringpi-gpiod\n");
    return 0;
}

void pwmToneWrite(int pin, int freq) {
    fprintf(stderr,
            LOG_WARN "pwmSetClock() called\n       "
                     "pwm is currently not supported by wiringpi-gpiod\n");
}

void pwmSetMode(int mode) {
    fprintf(stderr,
            LOG_WARN "pwmSetClock() called\n       "
                     "pwm is currently not supported by wiringpi-gpiod\n");
}

void pwmSetRange(unsigned int range) {
    fprintf(stderr,
            LOG_WARN "pwmSetClock() called\n       "
                     "pwm is currently not supported by wiringpi-gpiod\n");
}

void pwmSetClock(int divisor) {
    fprintf(stderr,
            LOG_WARN "pwmSetClock() called\n       "
                     "pwm is currently not supported by wiringpi-gpiod\n");
}

void gpioClockSet(int pin, int freq) {
    fprintf(stderr,
            LOG_WARN "gpioClockSet() called\n       "
                     "Clocks are currently not supported by wiringpi-gpiod\n");
}

unsigned int digitalReadByte(void) {
    int      pin, x;
    uint32_t raw;
    uint32_t data = 0;

    for (pin = 0; pin < 8; ++pin) {
        x    = digitalRead(pin);
        data = (data << 1) | x;
    }

    return data;
}

unsigned int digitalReadByte2(void) {
    int      pin, x;
    uint32_t data = 0;

    for (pin = 20; pin < 28; ++pin) {
        x    = digitalRead(pin);
        data = (data << 1) | x;
    }

    return data;
}

void digitalWriteByte(const int value) {
    uint32_t pinSet = 0;
    uint32_t pinClr = 0;
    int      mask   = 1;
    int      pin;

    for (pin = 0; pin < 8; ++pin) {
        digitalWrite(pin, value & mask);
        mask <<= 1;
    }
    return;
}

void digitalWriteByte2(const int value) {
    register int mask = 1;
    register int pin;

    for (pin = 20; pin < 28; ++pin) {
        digitalWrite(pin, value & mask);
        mask <<= 1;
    }
    return;
}

/*
 *  Interrupts (Also Pi hardware specific)
 */

int waitForInterrupt(int pin, int mS) {
    fprintf(stderr, LOG_WARN
            "waitForInterrupt() called\n       "
            "Interrupts are currently not supported by wiringpi-gpiod\n");
    return 0;
};

int wiringPiISR(int pin, int mode, void (*function)(void)) {
    fprintf(stderr, LOG_WARN
            "wiringPiISR() called\n       "
            "Interrupts are currently not supported by wiringpi-gpiod\n");
    return 0;
};

/*
 *  Threads
 */

static pthread_mutex_t piMutexes[4];

int piThreadCreate(void *(*fn)(void *)) {
    pthread_t myThread;
    return pthread_create(&myThread, NULL, fn, NULL);
}

void piLock(int key) { pthread_mutex_lock(&piMutexes[key]); }
void piUnlock(int key) { pthread_mutex_unlock(&piMutexes[key]); }

/*
 *  Schedulling priority
 */

int piHiPri(const int pri) {
    struct sched_param sched;

    memset(&sched, 0, sizeof(sched));

    if (pri > sched_get_priority_max(SCHED_RR))
        sched.sched_priority = sched_get_priority_max(SCHED_RR);
    else
        sched.sched_priority = pri;

    return sched_setscheduler(0, SCHED_RR, &sched);
}

/*
 *  Extras from arduino land
 */

// wait for n milliseconds
void delay(unsigned int howLong) {
    struct timespec sleeper, dummy;

    sleeper.tv_sec  = (time_t)(howLong / 1000);
    sleeper.tv_nsec = (long)(howLong % 1000) * 1000000;

    nanosleep(&sleeper, &dummy);
}

void delayMicrosecondsHard(unsigned int howLong) {
    struct timeval tNow, tLong, tEnd;

    gettimeofday(&tNow, NULL);
    tLong.tv_sec  = howLong / 1000000;
    tLong.tv_usec = howLong % 1000000;
    timeradd(&tNow, &tLong, &tEnd);

    while (timercmp(&tNow, &tEnd, <))
        gettimeofday(&tNow, NULL);
}

// wait for n microseconds
void delayMicroseconds(unsigned int howLong) {
    struct timespec sleeper;
    unsigned int    uSecs = howLong % 1000000;
    unsigned int    wSecs = howLong / 1000000;

    /**/ if (howLong == 0)
        return;
    else if (howLong < 100)
        delayMicrosecondsHard(howLong);
    else {
        sleeper.tv_sec  = wSecs;
        sleeper.tv_nsec = (long)(uSecs * 1000L);
        nanosleep(&sleeper, NULL);
    }
}

unsigned int millis(void) {
    uint64_t now;

    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    now = (uint64_t)ts.tv_sec * (uint64_t)1000 +
          (uint64_t)(ts.tv_nsec / 1000000L);

    return (uint32_t)(now - epochMilli);
}

unsigned int micros(void) {
    uint64_t now;

    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    now =
        (uint64_t)ts.tv_sec * (uint64_t)1000000 + (uint64_t)(ts.tv_nsec / 1000);

    return (uint32_t)(now - epochMicro);
}
