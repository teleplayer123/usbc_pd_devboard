### Description
A development board to experiment with USB-C PD via an STM32 and FUSB302. 

The [PD Logger](firmware/pd_logger) firmware logs PD messages, attach/dettach events, CC voltage/orientation, and VBUS voltage. 
Connect to the board through UART and use a serial terminal to interact with 
the FUSB302 over I2C via the STM32F0. Logs will start showing once a PD capable device is detected.
A debug console menu can be accessed by pressing Enter in the terminal allowing direct writes and
reads to the FUSB302. Press "q" to exit the debug console and return to live logging.

### Dev Board
![DevBoardPreview](assets/pd_board.png)

![DevBoard](assets/real_board.jpg)