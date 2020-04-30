# TinyUPS - Uninterruptible Power Supply based on ATtiny13A

# 1. Overview #

TinyUPS is a simple 5V/2.5A uninterruptible power supply with a li-ion battery as a buffer, a load sharing power path management system and an ATtiny13 for monitoring power supply and battery charge level as well as for communication with the connected device.

![IMG_20200429_085806_x.jpg](https://image.easyeda.com/pullimage/2XsCuAdfqt9wIi8S0v06BNLV0mbNk2amUA6AiitS.jpeg)
![IMG_20200429_085845_x.jpg](https://image.easyeda.com/pullimage/ehw9MdOgYZ1FBT7VzwjljH5wGpZlaYVFoSfBCoGl.jpeg)
![IMG_20200429_085907_x.jpg](https://image.easyeda.com/pullimage/kT6kqbDiSMt34Geqa3ajQAtOuR0XCEJVwFSN0zOP.jpeg)

# 2. Working Principle #

If external power is connected to the tinyUPS the input voltage or vcc of the ATtiny13 is delivered by this source, otherwise by the battery. The ATtiny13 monitors the input voltage and tells the connected device to shutdown by pulling the SHUTDOWN-line low when the input voltage falls below a certain threshold (SHUTDOWNLEVEL). This happens when the external power source is diconnected or disabled and the battery level falls below this threshold. After waiting a certain time (SHUTDOWNTIMER) to allow the connected device to safely shut down, the ATtiny13 deactivates the boost converter and turns off the power to the connected device.
If the input voltage rises again above a certain threshold (POWERONLEVEL) it activates the boost converter and turns on the power to the connected device. This happens when the external power source is available again.
When power is turned on a BOOTUPTIMER starts to count. If a shutdown is initiated before the boot up is completed, the left-over time is added to the SHUTDOWNTIMER in order to allow the connected device to completely boot up and shut down.
A shutdown can also be initiated by pressing and holding the button or by setting the REQUEST-line to high (>0.7V) for 2 seconds. After such shutdowns the power will not be turned on again automatically. The power to the connected device can be turned on manually by pressing the button or setting the REQUEST-line to high if the battery level is above a certain threshold (USERPOWERLEVEL) or the external power source is connected.

The SHUTDOWN pin of the tinyUPS is an open collector output. The connected device must have an internal or external pullup resistor on the SHUTDOWN line! This is necessary because of the different voltage levels.

The ATtiny13 spends most of the time in power-down sleep mode to save energy. The watch dog timer wakes it up every 8 seconds. It will also wake up if the button was pressed or the REQUEST-line was changed (pin change interrupt). After doing its stuff the ATtiny13 sleeps again. The current status of the tinyUPS is indicated by 5 LEDs:

|LED|State|
|-|-|
|VIN: on|external power is connected|
|CHARGE: on|battery is charging|
|FULL: on|battery is fully charged (is only shown if external power is connected)|
|STATUS: steady on|normal power-on operation|
|STATUS: blinking|in shutdown sequence|
|STATUS: short flashes|in standby (short flash occurs every 8 seconds)|
|VOUT: on|output power is turned on|

Although it would be possible to supply the connected device via the battery and charge the battery at the same time, this is absolutely not a recommended way. In this case, most charging ICs such as the TP4056 are unable to determine whether the battery is fully charged. The battery would be charged forever, which would destroy it in the long run. A load sharing system was therefore integrated, which separates the battery from the load when an external power is present. While the battery is being charged, the connected device is powered by the external power supply. For more details on the working principle of the load sharing power path management circuit refer to http://ww1.microchip.com/downloads/en/appnotes/01149c.pdf.

# 3. Performance #

External power supply should be capable of delivering enough power to charge the battery and to power the connected device simultaneously. The maximum battery charging current is set to 1000mA but you can set a lower limit by selecting a different value of R3. The output voltage of the external power supply must not exceed 5.2V! Choose a good 18650 Li-Ion battery with a low internal resistance which is capable of delivering up to 6A!

|Parameter|Value|
|-|-|
|Input Voltage|4.5 - 5.2 V|
|Output Voltage|4.8 - 5.2 V|
|Output Current|Max 2.5 A|
|Standby Current|95 uA|

![tinyUPS_bat_efficiency.png](https://image.easyeda.com/pullimage/W959GNvB4wwInaBowrjAbtAAMT8vCeLyaqSE4I29.png)
![tinyUPS_ext_efficiency_2.png](https://image.easyeda.com/pullimage/r1j2fOvBXxYUyLE3E1PBskUNkwubG3GuHiYs1wiT.png)
![tinyUPS_extpwroff.png](https://image.easyeda.com/pullimage/bh1e51RDliqaCUedjc1ODjaZxg4zE4oq562P07h2.png)
![tinyUPS_extpwron.png](https://image.easyeda.com/pullimage/vAquKosg7Vafx468a0X0v73O01uWxJlmcDbXxqjH.png)
![tinyUPS_ripple.png](https://image.easyeda.com/pullimage/Oku4Ay6szQZlZ7JVQbMMSMXAQoVwiRxVshBPpwjk.png)
