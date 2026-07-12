I tried with esp32-h2 but could not manage to get low power so got my self a nordic nrf52840 dongle and programmed it. This is two float sensor.

Used nrfProgrammer to write to the dongle
in build select no sys build
use toolkit 3.1.1 

when you clone also need to run this command in the west zap-generate



or use online qr generaters for matter. 

I struggled how to add an additional dongle so found info on this website 

https://devzone.nordicsemi.com/f/nordic-q-a/118484/how-to-modify-matter-device-s-discriminator-and-setup-pin-code-without-configuring-factorydata

You can change the descriminator with CONFIG_CHIP_DEVICE_DISCRIMINATOR in prj.conf. As for the PIN code, you can change it using CONFIG_CHIP_DEVICE_SPAKE2_PASSCODE, but you will also have to generate a new SPAKE2+ verifier for the new passcode and set CONFIG_CHIP_FACTORY_DATA_GENERATE_SPAKE2_VERIFIER=n.

The SPAKE2+ verifier can be generated with the SPAKE2+ Python Tool using the passcode, SPAKE2+ salt, and iteration count. The simplest is to use the default SPAKE2+ salt (U1BBS0UyUCBLZXkgU2FsdA==) and iteration count (1000). Here is an example where I use the default salt and iteration count but a different passcode:

CONFIG_CHIP_DEVICE_DISCRIMINATOR=0x231
CONFIG_CHIP_DEVICE_SPAKE2_PASSCODE=20372039
CONFIG_CHIP_FACTORY_DATA_GENERATE_SPAKE2_VERIFIER=n
CONFIG_CHIP_DEVICE_SPAKE2_TEST_VERIFIER="kLXow6J5EYWRa3aO71DkTG5qMtkn5PfwwXhOixGa2zEEDEDGlKfe7mEBAg6b3CTx++XPKapVRxnuWs6r6O90+7aDXjlDytFVwlS+PVpSY3dQTmm1/0aZKr9Mw+4mWPBefQ=="

Another big issue I have that took so long to fix is when you add another dongle i couldn't add it to HA. 

this is the default example chip-tool payload generate-qrcode --passcode 20202021 --discriminator 291 --vendor-id 65521 --product-id 32768 --commissioning-mode 0
