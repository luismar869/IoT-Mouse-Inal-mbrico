# ESP32 Mouse BLE
 
Mouse inalámbrico por Bluetooth Low Energy usando una ESP32. Los botones y direcciones se controlan con GPIOs físicos.
 
## Requisitos
 
- ESP-IDF v5.x instalado ([guía oficial](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/))
- ESP32 con soporte BLE
- Python 3.8+
 
## Compilar y flashear
 
```bash
# 1. Entra a la carpeta del proyecto
cd nombre-del-proyecto
 
# 2. Configura el target
idf.py set-target esp32
 
# 3. Compila
idf.py build
 
# 4. Flashea 
idf.py -p /dev/ttyUSB0 flash
 
# 5. Monitorea la salida serial
idf.py -p /dev/ttyUSB0 monitor
```
 
En Windows el puerto es tipo `COM3`. 
 
## Uso
 
1. Flashea y enciende la ESP32.
2. En tu PC o celular busca dispositivos Bluetooth y conecta **ESP32_Mouse**.
3. Presiona los botones para mover el cursor o hacer clic.

 