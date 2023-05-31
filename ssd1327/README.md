fb driver for ssd1327

Build
---------------------------
Modify the KERN_DIR in Makefile and ARCH,CROSS_COMPILE env val.

DT Node example
---------------------------
```
/*
   pin19: GPIO1_C1
   pin21: GPIO1_C0
   pin23: GPIO1_C2
   pin24: GPIO1_C4
*/

&spi4 {
        status = "okay";
        pinctrl-names = "default";
        pinctrl-0 = <&spi4m0_cs1 &spi4m0_pins>;
        assigned-clocks = <&cru CLK_SPI4>;
        assigned-clock-rates = <200000000>;
        num-cs = <2>;

        spi_dev@1 {
                status = "disabled";
                compatible = "rockchip,spidev";
                reg = <1>;
                spi-max-frequency = <50000000>;
        };

        uc8253: uc8253@0 {
                #address-cells = <1>;
                #size-cells = <1>;
                compatible = "ultrachip,uc8253";
                reg = <1>;
                spi-max-frequency = <10000000>;
                buswidth = <8>;
                xres = <360>;
                yres = <240>;
                spi-cpol;
                spi-cpha;
                //gpios = <&gpio1 RK_PA2 GPIO_ACTIVE_HIGH>;
                reset-gpios = <&gpio1 RK_PA3 GPIO_ACTIVE_HIGH>;
                dc-gpios = <&gpio2 RK_PD4 GPIO_ACTIVE_LOW>;
                debug = <1>;
                status = "okay";
        };

};

```

Installing
---------------------------
```
insmod ssd1327_fb.ko
```
or though kernel built-in

Gamma setting
---------------------------

```
# this node taken gamma values from 0x00 to 0xFF. More higher means more brighter
echo 0 > /sys/class/graphics/fb0/device/gamma
echo 128 > /sys/class/graphics/fb0/device/gamma
echo 255 > /sys/class/graphics/fb0/device/gamma
```