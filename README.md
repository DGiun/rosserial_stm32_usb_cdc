# ROSserial on STM32 USB

ROSserial for `STM32`, developed to work with [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) 1.8.0 projects.

Heavily based on [xav-jann1's rosserial_stm32f4](https://github.com/xav-jann1/rosserial_stm32f4) and [yoneken's rosserial_stm32](https://github.com/yoneken/rosserial_stm32).

## Usage
1. Create a new STM32CubeIDE project :
    - Choose `C++` as `Targeted Language`
    - Choose `Yes` for "Initialize all peripherals with their default Mode ?"
    
2. Configure microcontroller (Full Speed USB)
    - USB_OTG_FS -> Mode -> Device_Only
    - USB_DEVICE -> Class For FS IP -> Communication Device Class (Virtual Port Com)
    - Generate code

3. Create `ROS` libraries in your project :
    ```sh
    $ cd your/catkin_ws/src
    $ git clone https://github.com/DGiun/rosserial_stm32_usb_cdc
    $ cd ..
    $ catkin_make
    $ source devel/setup.bash
    $ rosrun rosserial_stm32_usb_cdc make_libraries.py ${path/to/your/stm32/project/Core}
    ```
    (`rosserial` should already be installed, if not : `sudo apt-get install ros-<distro>-rosserial`)

4. Add default paths for compilation :
    - Open `Project / Properties` window
    - Add in `C/C++ Build / Settings / Tool Settings / MCU G++ Compiler / Include paths` : 
        `../Core/Inc/ros_lib`, `../USB_DEVICE/Target`, `../USB_DEVICE/App`, `../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc`,        
        `../Middlewares/ST/STM32_USB_Device_Library/Core/Inc`
    - Add in `C/C++ General / Paths and Symbols / Source Location` : `Middlewares`, `USB_DEVICE`

5. Change Functions in `./USB_DEVICE/App/usbd_cdc_if.c`
    ```c
    /* USER CODE BEGIN PV */
    /* Private variables ---------------------------------------------------------*/
    uint32_t rx_head = 0u;
    uint32_t tx_head = 0u;
    /* USER CODE END PV */
    ```
    ```c
    static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
    {
        /* USER CODE BEGIN 6 */
        rx_head += *Len;
        USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
        USBD_CDC_ReceivePacket(&hUsbDeviceFS);
        return (USBD_OK);
        /* USER CODE END 6 */
    }
    ```
    ```c
    uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
    {
        uint8_t result = USBD_OK;
        /* USER CODE BEGIN 7 */
        USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

        memcpy(UserTxBufferFS + tx_head, Buf, Len);
        tx_head += Len;

        if (hcdc->TxState != 0)
            return USBD_BUSY;

        USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, tx_head);
        result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
        tx_head = 0;
        /* USER CODE END 7 */
        return result;
    }
    ```
6. Rename main.c -> main.cpp (  if you will renamed main.cpp -> main.c before Change Project.ioc  )
---
## Run
```sh
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=921600
```
---
## Exemples

See a [simple example](./example) with a `Publisher` and a `Subscriber`.

Also, see [yoneken's examples](https://github.com/yoneken/rosserial_stm32/tree/master/src/ros_lib/examples).

---
