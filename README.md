# dataLoggerNano33BLE

Arduino Nano 33 BLE data logger with BLE monitoring and remote control

Arduino Nano 33 BLE registrador de datos con monitoreo y control remoto usando BLE


Implementa un prototipo simple de data logger con la Arduino Nano 33 BLE Sense.
Componentes:

* Arduino Nano 33 BLE Sense
* GPS ublox NEO 7m
* SD reader
* Protoboard

<img src="https://github.com/droyktton/dataLoggerNano33BLE/blob/main/circuito.jpg?raw=true" width="500">

Características

* Resolución de lat y lon a la millonésima de grado.
* Imprime la Fecha y Hora (UTC) que da el gps.
* BLE para controlar/averiguar remoto!: 
  * La red es "Tortuga Nano" y uno le puede mandar comandos para averiguar/cambiar cosas, haciendo "write" de estos strings (enLightBlue)
  * "sd" : te dice si esta escribiendo o no en la sd
  * "ble": te dice si esta mandando los mismos datos por ble.
  * "sdon": activa la escritura en la sd.
  * "sdoff": desactiva la escritura en la sd.
  * "bleon": activa en el envío de los mismos datos en ble.
  * "bleoff": desactiva en el envío de los mismos datos en ble.
 
Apenas uno prende, está todo desactivado, pero imprime todos los datos en el serial monitor, así que tener celular a mano y lightblue para activar/desactivar escritura sd. Se pueden agregar comandos para averiguar el estado de la plaquita y activar/desactivar o ajustar otras cosas que vayamos agregando...

<img src="https://github.com/droyktton/dataLoggerNano33BLE/blob/main/demo.gif?raw=true" width="300">


Cosas que encontre utiles
 * Para ver en google earth el csv
   * gawk -F ',' '{print $1/1000000,",",$2/1000000}' GPSLOG.TXT > caminata.csv
   * y luego convertirlo a kml con esto
   * https://www.convertcsv.com/csv-to-kml.htm


![gps demo](https://github.com/droyktton/dataLoggerNano33BLE/blob/main/paseo.jpg?raw=true)

