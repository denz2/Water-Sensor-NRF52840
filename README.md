I tried with esp32-h2 but could not manage to get low power so got my self a nordic nrf52840 dongle and programmed it. This is two float sensor.

Used nrfProgrammer to write to the dongle
in build select no sys build
use toolkit 3.1.1 

when you clone also need to run this command in the west zap-generate

Another big issue I have that took so long to fix is when you add another dongle i couldn't add it to HA. 

so had to open up console and run the following command matter config discriminator 291  and then generate the code it didn't accept anything that I was putting in proj.conf

chip-tool payload generate-qrcode --passcode 20202021 --discriminator 291 --vendor-id 65521 --product-id 32768 --commissioning-mode 0

or use online qr generaters for matter. 



