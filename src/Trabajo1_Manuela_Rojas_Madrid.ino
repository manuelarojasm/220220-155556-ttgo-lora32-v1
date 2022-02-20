//incluir las librerias//
#include <TFT_eSPI.h> 
#include <SPI.h>
#include <UbiConstants.h>
#include <UbidotsEsp32Mqtt.h>
#include <UbiTypes.h>
#include "DHT.h"
#define DHTPIN 27 //definicion del pin al que se conecta el sensor 
#define DHTTYPE DHT11 //definir el tipo de dht 


TFT_eSPI tft = TFT_eSPI(); //se define el constructor de las funciones principales 
DHT dht (DHTPIN, DHTTYPE); //constructor 
const char *UBIDOTS_TOKEN = "BBFF-xFH0QzcOOz1K1j8upjtmWxfsx0t7MA";  // Put here your Ubidots TOKEN
const char *WIFI_SSID = "VILLASOL";      // Put here your Wi-Fi SSID
const char *WIFI_PASS = "701419041";      // Put here your Wi-Fi password
const char *DEVICE_LABEL = "esp32";   // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL1 = "Temperatura"; // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL2 = "Humedad"; // Put here your Variable label to which data  will be published
const char *SUBSCRIBE_DEVICE_LABEL = "esp32";   // Replace with the device label to subscribe to
const char *SUBSCRIBE_VARIABLE_LABEL1 = ""; // Replace with the variable label to subscribe to
const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds
 char tmp[12]; //arreglo de char para almacenar el float convertido 

unsigned long timer;
uint8_t analogPin = 27; // Pin used to read data from GPIO34 ADC_CH6.

Ubidots ubidots(UBIDOTS_TOKEN);
/****************************************
 * Auxiliar Functions
 ****************************************/

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

/****************************************
 * Main Functions
 ****************************************/


void setup() {
  
  // codigo pantalla:
  tft.init(); //se incializa la pantalla
  tft.fillScreen(0x0000); //llena la pantalla con color negro 
  //inicializacion sensor 
  Serial.begin(115200); //inicicalizar la comunicacion serial 
  Serial.println (F("DHTxx test!"));
  dht. begin();
  Serial.begin(115200);
  // ubidots.setDebug(true);  // uncomment this to make debug messages available
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  timer = millis();
  

}

void loop() {
  // codigo lectura sensor:
  delay (2000); //reatrdo de 2seg
  float h= dht.readHumidity(); //lee la humedad en fnc read y la almacena en h 
  float t= dht.readTemperature(); //lee la temperatura en fnc read y la almacena en t 
  if (isnan(h) || isnan(t)){
    Serial.println(F("Failed to read from DHT sensor!")); //isnan nos devuelve un 1 en caso de ue exista un fallo o un error en la lectura de la vble
    return; 
  }
   //imprime la variable en consola//
  Serial.print(F("Humedad: "));
  Serial.print(h);
  //imprime la variable en la pantalla//
  tft.drawString("Humedad:",40,10,2); //Y=VERTICAL 240 X=HORIZONTAL 135
  tft.drawString(dtostrf(h,2,2,tmp), 40, 40, 4); //convierte el valor de float a string
  //void  fillCircle(int32_t 70, int32_t 60, int32_t 20, uint32_t green); //se dibuja el primer circulo- switch
  //void  fillCircle(int32_t x, int32_t y, int32_t 20, uint32_t red),
  
  //imprime la variable en consola//
  Serial.print(F(" % temperatura: "));
  Serial.print(t);
  Serial.print(F("°C"));
  //imprime la variable en la pantalla//
  tft.drawString("Temperatura:",20,100,2); //usando la funcion de tft como string 
  tft.drawString(dtostrf(t,2,2,tmp), 40,120, 4); //covierte el valor de float a string usando dtostrf(float_value, min_width, num_digits_after_decimal, where_to_store_string)

   // codigo ubidots:
  if (!ubidots.connected())
  {
    ubidots.reconnect();
  }
  if (abs(millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  {
    float value = analogRead(analogPin); //se lee el pin analogo del sensor con la vble temperatura 
    float value2 = analogRead(analogPin); //se lee el pin analogo del sensor con vble humedad 
    Serial.print(value);
    Serial.println(); //salto de linea
    ubidots.add(VARIABLE_LABEL1, t); // Insert your variable Labels and the value to be sent
    ubidots.add(VARIABLE_LABEL2, h); // Insert your variable Labels and the value to be sent
    ubidots.publish(DEVICE_LABEL);
    timer = millis();
  }
  ubidots.loop();
}
