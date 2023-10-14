# `wiringpi-gpiod`

_A generic wiringPi-implementation for single-board computers._

## About

This is a lightweight re-implementation of the [wiringPi](https://github.com/WiringPi/WiringPi)-library based on gpiod – the [linux kernel GPIO API](https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/commit/?id=79a9becda8940deb2274b5aa4577c86d52ee7ecb).

This allows for `wiringpi-gpiod` to be used on any single-board computer, as long as the kernel recognizes the GPIO chip – the original wiringPi-library supported only a [limited range](http://web.archive.org/web/20231011062333/http://wiringpi.com/#post-15) of SBCs.

## Compiling

```bash
> cmake -B build .
> cmake --build build
# produces build/libwiringpi-gpiod.[a|so]

# [optional]
> sudo cmake --install build
```

### And linking against it

Linking a wiringPi-based project against `wiringpi-gpiod` is as simple as replacing `-lwiringpi` with `-lwiringpi-gpiod` in the compiler arguments.

In case you do not wish to install the library system-wide, add an additional `-L/path/to/libwiringpi-gpiod.so`.

## Dependencies

* [gpiod-dev](https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git/tree/)

## Comparison to the original wiringPi-library

* Runs on any single-board computer, as long as GPIO pins are recognized [by the kernel](https://www.kernel.org/doc/html/latest/driver-api/gpio/board.html)
* ! Pin numbers in `wiringpi-gpiod` follow the standard GPIOxx-numbering of your board (\*not\* the [custom wiringPi-mapping](https://pinout.xyz/pinout/wiringpi))

---
Created by Lesosoftware in 2023
