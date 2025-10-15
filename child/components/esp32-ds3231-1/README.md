# DS3231 RTC Library for ESP32 & ESP-IDF

This library provides low-level support for interfacing
with a DS3231 high-precision real time clock via I2C
on an Espressif ESP32 system using the ESP-IDF development
platform.

## Basic Features

This library's support for the DS3231's feature set is limited
to core timekeeping functions. The bells and whistles - such
as signal generation and temperature sensing - are not supported.
For a richer feature set, check out the library from which this
library was derived,
[DS3231 by Uncle Rus](https://github.com/UncleRus/esp-idf-lib/blob/master/components/ds3231).

## Supported Ecosystem

This library aims to support the following... and only the following:

* **Microcontroller:** Espressif [ESP32 modules](https://www.espressif.com/en/products/modules/esp32)
* **IC:** DS3231
* **Interface:** I2C
* **Development Platform:** [ESP-IDF v4.1+](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/index.html),
  Espressif's freeRTOS-based platform for ESP32 development

## Installation

Using the modern ESP-IDF with CMake, Git and the recommended
project structure (with a `components` folder), the easiest way
to install this library is to add it to the `components` folder
as a Git submodule:

```shell
cd ${YOUR_PROJECT_ROOT}/components
git submodule add https://github.com/mvolk/esp32-ds3231.git
git commit -m "Add esp32-ds3231 component"
```

## API

See [ds3231.h](./include/ds3231.h) for detailed documentation of
the following API features:

void ds3231_init_info(DS3231_Info *ds3231, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint32_t timeoutMs);

* `ds3231_set_time(...)`
* `ds3231_get_time(...)`
* `ds3231_get_oscillator_stop_flag()`
* `ds3231_clear_oscillator_stop_flag()`

## Support

Technical support is not available.

## Contributing

### Bug Reports

Please feel free to report bugs by
[opening a new issue via Github](https://github.com/mvolk/esp32-ds3231/issues/new).

Please do not use Github issues to request technical support.

### Pull Requests

Pull requests that further the goals of this library are welcome.

The [issues list](https://github.com/mvolk/esp32-ds3231/issues) is a
good place to look for ideas if you would like to contribute but don't
have a specific contribution in mind.

If you would like to extend support to additional MCUs or development
platforms, please consider forking or building on top of this library.

## Credits

This library is a stripped-down version of a
[ds3231 library by Uncle Rus](https://github.com/UncleRus/esp-idf-lib/blob/master/components/ds3231),
which in turn was ported from `esp-open-rtos` and the
[i2cdev library by Uncle Rus](https://github.com/UncleRus/esp-idf-lib/blob/master/components/i2cdev).

## License & Copyright

See [LICENSE.txt](./LICENSE.txt) for license details.

## Liability Notice

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
