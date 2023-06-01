<h1 align="center">Suniv Epd</h1>

<h3 align="center">
So, this is it.
</h3>

<p align="center">
SSD1327 based 128x128 OLED display may the best choice i have ever made.
</p>

<p align="center">
It has good refresh rate, low power consumption and adjustable contrast.
</p>

<p align="center">
This will be the best place to develop compatible display driver for my new project "NINJAR".
</p>

<p align="center">
Anyway, no matter what the result is，[ I leave this reason for you ]. okay?
</p>

------------------------

update notes After 6/1/2023

<span>
    Heya, I'm developing linux drivers for E-paper display recently.
</span>

</br>

<span>
    When I was developing fb driver for st7789v, there some problem with
    spi signal caused device received wrong color data.
</span>

</br>

<span>
    The fb driver for ssd1306 isn't a real grayscale format, i mean it's convert from rgb565, learn nothing. so keep develop grayscale driver.
</span>

</br>

<span>
This project is based on fbtft
</span>