// Library includes
#include <Arduino.h>
#include <math.h>
#include <esp_sleep.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/queue.h"
#include "esp_wifi.h"

#define LED_BUILTIN 3
const int ledPin = LED_BUILTIN;

// Local includes 
#include "parameters.h"
// #include "file_func.h"
#include "connectivity.h"
#include "dimensions.h"
//#include "mqtt.h"

// We need our utils functions for calculating MAD - Autoencoder
extern "C" {
#include "utils.h"
};

// Set debug info
#define DEBUG 5 //{0: No debug anything, 
                // 1: Debug full DQ and Autoencoder results, 
                // 2: Debug resume DQ only, 
                // 3: Debug resume Autoencoder only,
                // 4: Debug DQ and Autoencoder resume,
                // 5: Debug time and memory}

#define MAX_SIZE 60

// Execution times
char mensaje[200];
int posicion = 0;

// Sleep function with low power consumption and wifi sleep, no disconnection.
//esp_wifi_set_ps(WIFI_PS_MAX_MODEM); // Configurar el modo de bajo consumo Modem Sleep
void light_sleep(int seg){
  // Sleep modes ESP32 https://deepbluembedded.com/esp32-sleep-modes-power-consumption/ 
  // Sleep modes ESP32 https://lastminuteengineers.com/esp32-sleep-modes-power-consumption/
  esp_sleep_enable_timer_wakeup(seg * 1000000); // Espera seg segundos
  esp_wifi_set_ps(WIFI_PS_MAX_MODEM); // Configurar el modo de bajo consumo Modem Sleep
  esp_light_sleep_start();
}

void changeFrecuency(int seg){

  // Consumo en diferencias frecuencias https://mischianti.org/esp32-practical-power-saving-manage-wifi-and-cpu-1/

  delay(1000);
  //Serial.print(F("Current CPU Frecuency: "));
  //Serial.println(getCpuFrequencyMhz());
  setCpuFrequencyMhz(80);
  //Serial.print(F("Now CPU Frecuency: \n"));
  //Serial.println(getCpuFrequencyMhz());
  delay(seg * 1000);
  setCpuFrequencyMhz(240);
  //Serial.print(F("Restored CPU Frecuency: \n"));
  //Serial.println(getCpuFrequencyMhz());
  delay(1000);
}

// Settings DQ
extern PubSubClient client;
extern String in_txt;
extern bool callback;


//int listSize;  // Tamaño de la lista que almacena los values
//int startline; // Inicializa la lectura desde la línea 0
//int siataValue; // Contador para extraer el valor de la estación SIATA

bool ban = true;
int frec = 1000; // Espacio de tiempo entre los values que llegan (en milisegundos)
static float values_df[MAX_SIZE];
static float values_nova[MAX_SIZE];
float value_siata;

void value_to_list(float *list, String value, int pos ){
    if (value.equalsIgnoreCase("nan")) {
        list[pos] = NAN;  // Representación de NaN en C
    } else {
        list[pos] = value.toFloat();
    }
}

/*
void value_to_list(float *list, const char* value, int pos ){
    if (strcmp(value, "nan") == 0)  {
        list[pos] = NAN;  // Representación de NaN en C
    } else {
        list[pos] = atof(value);
    }
}
*/

// Declaración de una cola
//QueueHandle_t queue_df;
//QueueHandle_t queue_nova;

void task1(void *parameter) {
  int cont = 0; // Variable de conteo de datos recibidos.
  float queue_df[MAX_SIZE];
  float queue_nova[MAX_SIZE];
  int comaPos1;
  String df_value;
  int comaPos2;
  String nova_value;
  //Serial.print(F("************ Free Memory: (Memoria inicial)"));
  //Serial.println(esp_get_free_heap_size());

  callback = true;

  while(true){
    delay(frec/4);
    client.loop();

    if (cont > 59){
      callback = false;
      cont = 0;
      memcpy(values_df, queue_df, sizeof(queue_df));
      memcpy(values_nova, queue_nova, sizeof(queue_nova));
      ban = false;
       //light_sleep(3);
    }

    if (callback){
      ledBlink(1);

      in_txt += datos_df[cont];
      in_txt += ",";
      in_txt += datos_nova[cont];
      in_txt += ",";
      in_txt += dato_siata;

      if (in_txt.length() > 1){     

        //   Encontrar la posición de la primera coma
        comaPos1 = in_txt.indexOf(',');

        // Extraer el token antes de la coma
        df_value = in_txt.substring(0, comaPos1);

        comaPos2 = in_txt.indexOf(',', comaPos1+1);
        nova_value = in_txt.substring(comaPos1+1,comaPos2);
        
        in_txt = in_txt.substring(comaPos2 + 1);
        value_siata = in_txt.toFloat();

        in_txt = "";


      //   if (token != ID){
        /*
        printf("Valor DF: %s\n",df_value);
        printf("Valor NOVA: %s\n",nova_value);
        printf("Valor SIATA: %s\n",in_txt);
        printf("%d\n",cont);
        printf("\n");
        */

        value_to_list(queue_df, df_value, cont);
        value_to_list(queue_nova, nova_value, cont);      

        //callback = false;
        delay(500);
        cont++;
      } else{
        cont = 60;
      }
      //cont++;
      
    }
  }
}

void task2(void *parameter) {
  //float dimen[24][10];
  char outlier;
  float mae_loss;
  int decimales = 5;
  size_t listSize;
  static float input_data[MAX_SIZE];
  static float valuesFusioned[MAX_SIZE];
  char dato[30]; //size of the number

  float p_com_df;   
  float p_com_nova;
  float uncer;
  float p_df;
  float p_nova;
  float a_df;
  float a_nova;
  float concor;
  float fusion;
  float DQIndex;
  float valueNorm; // value for normalized values as autoencoder input
  float acum; // Acumulator for Read predicted y value from output buffer (tensor)
  float pred_vals; // Value predicted for the Autoencoder


  #if DEBUG == 2
    Serial.print(F("Ecomp_df,Ecomp_nova,Epres_df,Epres_nova,Eacc_df,Eacc_nova,Euncer,Econcor,Efusion,EDQIndex\n"));
  #endif

  #if DEBUG == 3
    Serial.print(F("Evalue,EOUTLIER,Emae\n"));
  #endif

  #if DEBUG == 5
    Serial.print(F("Et_beforeDQ,Emem_beforeDQ,Et_afertDQ,Emem_afertDQ,Et_initAuto,Emem_initAuto,Et_finAuto,Emem_finAuto\n"));
  #endif

  while (true) {

    delay(frec/2);

    if(!ban){
      //Serial.print(F("************ Free Memory: (calculo DQ)"));
      //Serial.println(esp_get_free_heap_size());

      //Sleep
      //light_sleep(2);
      changeFrecuency(4);
      
      
      #if DEBUG == 1
        Serial.print(F("Tarea 2 ejecutándose en el núcleo 1\n"));
      #endif
      ban = true;

      listSize = sizeof(values_nova) / sizeof(values_nova[0]);
      //listSize = sizeof(values_nova)/4;

      #if DEBUG == 5
        // t_beforeDQ and mem_beforeDQ
        posicion += sprintf(mensaje + posicion, "%lu", micros());
        mensaje[posicion++] = ',';
        posicion += sprintf(mensaje + posicion, "%u", (ESP.getHeapSize() - ESP.getFreeHeap()));
        mensaje[posicion++] = ',';
      #endif

      p_com_df = completeness(values_df, listSize);   
      p_com_nova = completeness(values_nova, listSize);
      uncer = uncertainty(values_df, values_nova, listSize);
      p_df = precision(values_df, listSize);
      p_nova = precision(values_nova, listSize);
      a_df = accuracy(values_df, value_siata, listSize);
      a_nova = accuracy(values_nova, value_siata, listSize);
      concor = PearsonCorrelation(values_df, values_nova, listSize);
      plausability(p_com_df, p_com_nova, p_df, p_nova, a_df, a_nova, values_df, values_nova, listSize, valuesFusioned);
      fusion = calculateMean(valuesFusioned, listSize);
      DQIndex = DQ_Index(valuesFusioned, uncer, concor, value_siata, listSize);


      #if DEBUG == 5
        // t_afterDQ and mem_afterDQ
        posicion += sprintf(mensaje + posicion, "%lu", micros());
        mensaje[posicion++] = ',';
        posicion += sprintf(mensaje + posicion, "%u", (ESP.getHeapSize() - ESP.getFreeHeap()));
        mensaje[posicion++] = ',';
      #endif

      //Serial.print(F("Inicio Pausa\n"));
      //light_sleep(3);
      //Serial.print(F("Fin Pausa\n"));
      

      #if DEBUG == 1
        Serial.print(F("\n**********************************************\n"));
        Serial.print(F("********** Completeness DF: "));
        Serial.println(p_com_df, decimales);
        Serial.print(F("********** Completeness NOVA: "));
        Serial.println(p_com_nova, decimales);
        Serial.print(F("********** Uncertainty: "));
        Serial.println(uncer, decimales);
        Serial.print(F("********** Precision DF: "));
        Serial.println(p_df, decimales);
        Serial.print(F("********** Precision NOVA: "));
        Serial.println(p_nova, decimales);
        Serial.print(F("********** Accuracy DF: "));
        Serial.println(a_df, decimales);
        Serial.print(F("********** Accuracy NOVA: "));
        Serial.println(a_nova, decimales);
        Serial.print(F("********** Concordance: "));
        Serial.println(concor, decimales);
        Serial.print(F("********** Value Fusioned: "));
        Serial.println(fusion, decimales);
        Serial.print(F("********** DQ Index: "));
        Serial.println(DQIndex, decimales);
      #endif

      #if (DEBUG == 2) || (DEBUG == 4)
        Serial.print(p_com_df, decimales);
        Serial.print(F(","));
        Serial.print(p_com_nova, decimales);
        Serial.print(F(","));
        Serial.print(p_df, decimales);
        Serial.print(F(","));
        Serial.print(p_nova, decimales);
        Serial.print(F(","));
        Serial.print(a_df, decimales);
        Serial.print(F(","));
        Serial.print(a_nova, decimales);
        Serial.print(F(","));
        Serial.print(uncer, decimales);
        Serial.print(F(","));
        Serial.print(concor, decimales);
        Serial.print(F(","));
        Serial.print(fusion, decimales);
        Serial.print(F(","));
        Serial.println(DQIndex, decimales);
      #endif

      /*
      char mqtt_msg[50];
      sprintf(mqtt_msg, "%s,%.5f,distancia,%.5f",ID,fusion,DQIndex);
      client.publish(TOPIC.c_str(), mqtt_msg);

      char resultString[50];
      sprintf(resultString, "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f",p_com_df,p_com_nova,p_df,p_nova,a_df,a_nova,uncer,concor,fusion,DQIndex);
      write_text_to_file(dimensions, resultString);
      dimen[siataValue%24][0] = p_com_df;
      dimen[siataValue%24][1] = p_com_nova;
      dimen[siataValue%24][2] = p_df;
      dimen[siataValue%24][3] = p_nova;
      dimen[siataValue%24][4] = a_df;
      dimen[siataValue%24][5] = a_nova;
      dimen[siataValue%24][6] = uncer;
      dimen[siataValue%24][7] = concor;
      dimen[siataValue%24][8] = fusion;
      dimen[siataValue%24][9] = DQIndex;

      //read_data_from_file("/spiffs/data.txt"); // Lee el archivo con formato de hora y valor float
      */
      
      //Serial.print(F("************ Free Memory: (Autoencoder) "));
      //Serial.println(esp_get_free_heap_size());

     
      #if DEBUG == 5
        // t_initAuto and mem_initAuto
        posicion += sprintf(mensaje + posicion, "%lu", micros());
        mensaje[posicion++] = ',';
        posicion += sprintf(mensaje + posicion, "%u", (ESP.getHeapSize() - ESP.getFreeHeap()));
        mensaje[posicion++] = ',';
      #endif

    
      
      //light_sleep(4);
      
      
      #if DEBUG == 5
      // t_finAuto and mem_finAuto
        posicion += sprintf(mensaje + posicion, "%lu", micros());
        mensaje[posicion++] = ',';
        posicion += sprintf(mensaje + posicion, "%u", (ESP.getHeapSize() - ESP.getFreeHeap()));
        mensaje[posicion] = '\0';
        Serial.println(mensaje);
        mensaje[0] = '\0';
        posicion = 0;
      #endif

      changeFrecuency(4);

      ConnectToWiFi();
      createMQTTClient();

      delay(500);

      for (int i = 0; i < listSize; i++ ){

        sprintf(dato, "%s,%f,%f,%f", ID.c_str(),*(values_df + i),*(values_nova + i),value_siata);
        client.publish((TOPIC + 1).c_str(), dato);
        ledBlink(1);
        delay(500);
      }
      
      client.disconnect();
      WiFi.disconnect();
      
      changeFrecuency(4);

      callback = true;
      

      // Liberar la memoria asignada por normalize_data
      //free(input_data);
      //Serial.print(F("************ Free Memory: (Fin de ronda)"));
      //Serial.println(esp_get_free_heap_size());

    }

  }

    //printf("************ Free Memory task 2: %u bytes\n", esp_get_free_heap_size());

}

void setup() {
  Serial.begin(115200);

  // Set pin mode
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  

  // Crear la cola
  //queue_df = xQueueCreate(1, sizeof(float[60]));
  //queue_nova = xQueueCreate(1, sizeof(float[60]));

  // Configurar y conectar WiFi
  ConnectToWiFi();
  WiFi.disconnect();
  
  //initialize_spiffs();
  //create_file(data); // Archivo de memoria permanente 
  //create_file(dimensions); // Archivo que almacena las métricas cada hora 
  //write_text_to_file(dimensions, "hora,comp_df,comp_nova,prec_df,prec_nova,acc_df,acc_nova,uncer,concor");
  //write_text_to_file(data, "fechaHora,pm25df,pm25nova");
  //createMQTTClient();

  // Crea dos tareas y las asigna a diferentes núcleos
  xTaskCreatePinnedToCore(task1, "Task1", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(task2, "Task2", 10000, NULL, 1, NULL, 1);

  // Iniciar el planificador de tareas
  //vTaskStartScheduler();
}

void loop() {
  //reconnectMQTTClient();
  //client.loop();
  //delay(1000);
}