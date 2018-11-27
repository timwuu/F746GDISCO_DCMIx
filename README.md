# F746GDISCO_DCMI
AN5020 DCMI Example: STM32FGDISCOVERY with STM32F4DIS-CAM (OV9655)

IDE: Atollic TrueSTUDIO for STM32 

Notes:
1. use HSE 25MHz(X2) as the clock source
2. put font24.c in src folder, and no need to include the file in main.c
3. the LCD-TFT might burn if using the 320x240 resolution
4. some issues with STM32CubeMX code regeneration

2018.11.27 Working Ch 6.3.2
Remove unused modules in STM32CubeMX (which might cause problems)
