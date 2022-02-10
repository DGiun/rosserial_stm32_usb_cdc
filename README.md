# ROSserial on STM32

ROSserial for `STM32`, developed to work with [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) 1.8.0 projects.

Heavily based on [rosserial_stm32f4](https://github.com/xav-jann1/rosserial_stm32f4) and [yoneken's rosserial_stm32](https://github.com/yoneken/rosserial_stm32).

# Usage
## UART
1. Create a new STM32CubeIDE project :
    - Choose `C++` as `Targeted Language`
    - Choose `Yes` for "Initialize all peripherals with their default Mode ?"
    
2. Configure microcontroller (Full Speed USB)
    - USB_OTG_FS -> Mode -> Device_Only
    - USB_DEVICE -> Class For FS IP -> Communication Device Class (Virtual Port Com)
    - Generate code (+ [checking](#check-generated-code--very-important))

3. Create `ROS` libraries in your project :
    ```sh
    $ cd your/catkin/workspace/src
    $ git clone https://github.com/DGiun/rosserial_stm32_usb_cdc
    $ cd ..
    $ catkin_make
    $ source devel/setup.bash
    $ rosrun rosserial_stm32_usb_cdc make_libraries.py ${path/to/your/stm32/project/Core}
    ```
    (`rosserial` should already be installed, if not : `sudo apt-get install ros-<distro>-rosserial`)

4. Add `ros_lib` to the default paths for compilation :
- Open `Project / Properties` window
- Add in `C/C++ Build / Settings / Tool Settings / MCU G++ Compiler / Include paths` : 
`../Core/Inc/ros_lib` `../USB_DEVICE/Target` `../USB_DEVICE/App` `../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc` `../Middlewares/ST/STM32_USB_Device_Library/Core/Inc`

### Check generated code (! Very important)

This implementation use [`DMA`](https://embedds.com/using-direct-memory-access-dma-in-stm23-projects/) for the serial interface.

Sometimes, the generated code initialises `DMA` and `USART` in the wrong order in the file `main.c`.

To work correctly, `MX_DMA_Init()` should be before `MX_USART2_UART_Init()`. If not, add `MX_DMA_Init()` between the `Init` brackets :
```cpp
  /* USER CODE BEGIN Init */
  MX_DMA_Init();
  /* USER CODE END Init */
```

---
## Exemples

See a [simple example](./example) with a `Publisher` and a `Subscriber`.

Also, see [yoneken's examples](https://github.com/yoneken/rosserial_stm32/tree/master/src/ros_lib/examples).

---
## Usage for other STM32 series

This package can easily be modified to be used for other STM32 series, by updating only two files :

- In `ros_lib/BufferedSerial.hpp`'s includes, change `?` to the number of the serie :
    ```cpp
    #include <string.h>
    #include "stm32f?xx_hal.h"
    #include "stm32f?xx_hal_uart.h"
    ```

- In `ros_lib/STM32Hardware.h`, update the number of the `huart` used for the project :
    ```cpp
    #include "BufferedSerial.hpp"
    extern UART_HandleTypeDef huart?;

    // Create Serial Buffer with UART?:
    BufferedSerial buff_serial(huart?);
    ```
