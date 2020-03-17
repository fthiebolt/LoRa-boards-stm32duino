# LoRa boards stm32duino
stm32duino support for some LoRa boards like
  * BSFrance stm32l151c8t6 board featuring a sx1276 (LoRa) module
  * Heltec LoRa Node L151 featuring a stm32l151ccu6 + sx1276

## NEWS ##
 * **[Mar.20]** Heltec LoRa Node 151 is working :)

 *Note: stm32l151cc belongs to product_line:STM32L151xC ... that doesn't work ... weird GPIO behaviour ... but it's ok when applying STM32L151xB product_line :)  ---go to 'development' branch to test with STM32L151xC product_line*

 * **[apr.19]** BSFrance L151 board is working :)

### Getting started
First of all, you ought to install stm32duino in your Arduino IDE along with STM32CubeProgrammer

https://github.com/stm32duino/Arduino_Core_STM32

Then, run script './deploy.sh' and board will get added to your Arduino boards list :)

Please note that i don't use USB bootloader ... instead i prefer the serial one embedded within each stm32 :)
For such behaviour, you just need to activate GPIO0 during powerup/reset procedure.

### TODO
a lot ... for example nothing was done to provide USB support ...

Ultimately, my goal is to get rid of this repository and to have thoses boards supported within stm32duino toolchain :)
