#include <WiFi.h>
#include <WiFiClient.h>
#include <esp_camera.h>
#include <esp_http_server.h>
#include <WebServer.h>
#include "base64.h"
#include <ESPmDNS.h>
#include <WiFiUdp.h>

#include <ArduinoOTA.h> //senha admin
#include <Arduino.h>

// #include "SD_MMC.h"            // SD Card ESP32
// #include "soc/soc.h"           // Disable brownour problems
// #include "soc/rtc_cntl_reg.h"  // Disable brownour problems
// #include "driver/rtc_io.h"
// #include <EEPROM.h>            // read and write from flash memory
// #define EEPROM_SIZE 1
// int pictureNumber = 1;

#define LED_PIN 4 //pino LED alto brilho 4

#define BUTTONN 2 //pino botão 2

#define LED_PIN_R 13 //pino LED 13
#define LED_PIN_G 15 //pino LED 15
#define LED_PIN_B 14 //pino LED 14

bool Erro = false;


//ajuste da qualidade
camera_config_t config; //var global

int quality = 8; //qualidade da camera
int FPS_desejado = 12; //FPS desejado

// Defina as credenciais do ponto de acesso Wi-Fi
const char* ssid = "Attlas";
const char* password = "";

// Defina a porta do servidor de streaming
WebServer server(80);

// Inicializa a câmera
void initCamera(int quality_) {
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn = 32;

  
  config.pin_reset = -1;  // Sem reset externo
  config.xclk_freq_hz = 20000000; //2
  config.pixel_format = PIXFORMAT_JPEG;
  
  
  
  //init with high specs to pre-allocate larger buffers
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = quality_;//10;
  config.fb_count = 4;//2; //quatidade do buffer 

  /*
    FRAMESIZE_96X96: Quadro de 96x96 pixels.
    FRAMESIZE_QQVGA: Quadro de 160x120 pixels.
    FRAMESIZE_QCIF: Quadro de 176x144 pixels.
    FRAMESIZE_HQVGA: Quadro de 240x176 pixels.
    FRAMESIZE_240X240: Quadro de 240x240 pixels.
    FRAMESIZE_CIF: Quadro de 352x288 pixels.    bom tamanho
    FRAMESIZE_VGA: Quadro de 640x480 pixels.    bom tamanho
    FRAMESIZE_SVGA: Quadro de 800x600 pixels.
    FRAMESIZE_XGA: Quadro de 1024x768 pixels.
    FRAMESIZE_SXGA: Quadro de 1280x1024 pixels.
    FRAMESIZE_UXGA: Quadro de 1600x1200 pixels.
    */



  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Erro ao inicializar a câmera! (0x%x)\n", err);
    delay(500);
    err = esp_camera_init(&config);
    Erro = true;
  }
  else {
    Serial.printf("Câmera ok\n", err);
  }
}


void initWIFI(){
  //se existir a rede
  WiFi.begin(ssid, password);

  int tentativas = 12;
  while (WiFi.status() != WL_CONNECTED) {
    delay(600);
    Serial.println("Conectando à rede Wi-Fi...");

    tentativas = tentativas - 1;
    Serial.println(tentativas);
    if (tentativas == 0){
      break;
    }
  }

  if (tentativas != 0){
    Serial.println("Conexão Wi-Fi estabelecida!");
    Serial.print("Endereço IP: ");
    Serial.println(WiFi.localIP());

    Erro = false;
  }
  else 
  {
    Serial.println("Sem rede Wi-Fi");
    delay(500);
    Erro = true;

    //se não existir, criar AP
    //WiFi.softAP(ssid, password);
    //IPAddress IP = WiFi.softAPIP();
    //Serial.print("Endereço IP do servidor: ");
    //Serial.println(IP);
  }
}




void handleRoot() {
  sensor_t * s = esp_camera_sensor_get();
  WiFiClient client = server.client();

  // Send HTTP headers for video streaming
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println("");


  int frameCount = 0;
  unsigned long startTime = millis();

  bool travamento = true;

  while (true) {
    digitalWrite(LED_PIN_R, LOW);
    digitalWrite(LED_PIN_G, LOW);
    digitalWrite(LED_PIN_B, HIGH);

    // Take a new frame
    camera_fb_t * fb = esp_camera_fb_get();
    int tentativas = 5; 
    while (!fb) { 
      digitalWrite(LED_PIN_R, HIGH);
      Serial.println("Camera capture failed");
      delay(100);
      camera_fb_t * fb = esp_camera_fb_get();

      tentativas = tentativas - 1;
      if (tentativas == 0){
        digitalWrite(LED_PIN_R, LOW);
        break;
      }
      digitalWrite(LED_PIN_R, LOW);
    }

    // Send the frame as a multipart response
    client.println("--frame");
    client.println("Content-Type: image/jpeg");
    client.println("Content-Length: " + String(fb->len));
    client.println("");
    client.write(fb->buf, fb->len);
    client.println("");


    // initialize EEPROM with predefined size
    //EEPROM.begin(EEPROM_SIZE);
    //pictureNumber = EEPROM.read(0) + 1;

    // Path where new picture will be saved in SD Card
    //String path = "/picture" + String(pictureNumber) +".jpg";

    //fs::FS &fs = SD_MMC; 
    //Serial.printf("Picture file name: %s\n", path.c_str());
  
    //File file = fs.open(path.c_str(), FILE_WRITE);
    //if(!file){
    //  Serial.println("Failed to open file in writing mode");
    //} 
    //else {
    //  file.write(fb->buf, fb->len); // payload (image), payload length
    //  Serial.printf("Saved file to path: %s\n", path.c_str());
    //  EEPROM.write(0, pictureNumber);
    //  EEPROM.commit();
    //}
    //file.close();



    // Release the frame buffer
    esp_camera_fb_return(fb);


    frameCount++; // Increment frame count
    unsigned long currentTime = millis();
    if (currentTime - startTime >= 1000) { // Print FPS every second
      Serial.print("FPS: ");
      Serial.print(frameCount);
      Serial.print(", JPEG quality: ");
      Serial.print(quality);

      Serial.print(", Wifi RSSI: ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm");
      
      if ((quality > 5) && (quality < 55)){
        if(frameCount < 5){
          digitalWrite(LED_PIN_G, travamento);
          travamento = !travamento;
        }
        else
          travamento = false;

        if(frameCount < FPS_desejado){
          if(quality < 50){ //máximo da má qualidade
            quality++;
            s->set_quality(s, quality);
          }
        }
        else if(frameCount > FPS_desejado){
          if(quality > 10){ //máximo da boa qualidade
            quality--;
            s->set_quality(s, quality);
          }
        }
      }

      frameCount = 0;
      startTime = currentTime;
    }

    if(!client.connected()){
      Serial.println("cliente sumiu");
      digitalWrite(LED_PIN_R, LOW);
      digitalWrite(LED_PIN_G, HIGH);
      digitalWrite(LED_PIN_B, LOW);
      break;
    }
  }
}



void setup() {
  int tempo_espera = 200;
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTONN, INPUT);
  pinMode(LED_PIN_R, OUTPUT);
  pinMode(LED_PIN_G, OUTPUT);
  pinMode(LED_PIN_B, OUTPUT);


  
  for (int i = 0; i < 1; i++){
    Serial.print("Test init: ");
    Serial.println(i);

    digitalWrite(LED_PIN, LOW);
    digitalWrite(LED_PIN_R, LOW);
    digitalWrite(LED_PIN_G, LOW);
    digitalWrite(LED_PIN_B, LOW);
    Serial.print("Valor do botao: ");
    Serial.println(digitalRead(BUTTONN));


    // Conecta-se à rede Wi-Fi como ponto de acesso
    initWIFI();
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(LED_PIN_R, LOW);
    digitalWrite(LED_PIN_G, LOW);
    digitalWrite(LED_PIN_B, LOW);
    Serial.print("Valor do botao: ");
    Serial.println(digitalRead(BUTTONN));
    delay(50);


    digitalWrite(LED_PIN, LOW);

    digitalWrite(LED_PIN_R, HIGH);
    digitalWrite(LED_PIN_G, LOW);
    digitalWrite(LED_PIN_B, LOW);
    Serial.print("Valor do botao: ");
    Serial.println(digitalRead(BUTTONN));
    delay(500);

    digitalWrite(LED_PIN, LOW);

    digitalWrite(LED_PIN_R, LOW);
    digitalWrite(LED_PIN_G, HIGH);
    digitalWrite(LED_PIN_B, LOW);
    Serial.print("Valor do botao: ");
    Serial.println(digitalRead(BUTTONN));
    delay(500);

    initCamera(quality);

    digitalWrite(LED_PIN, LOW);

    digitalWrite(LED_PIN_R, LOW);
    digitalWrite(LED_PIN_G, LOW);
    digitalWrite(LED_PIN_B, HIGH);
    Serial.print("Valor do botao: ");
    Serial.println(digitalRead(BUTTONN));
    delay(500);

    digitalWrite(LED_PIN, LOW);
    digitalWrite(LED_PIN_R, LOW);
    digitalWrite(LED_PIN_G, LOW);
    digitalWrite(LED_PIN_B, LOW);
    Serial.print("Valor do botao: ");
    Serial.println(digitalRead(BUTTONN));
  }

  // Inicia o servidor HTTP para streaming
  // Start the server
  server.on("/", handleRoot);
  server.begin();
  Serial.println("Server started");


  //cartão SD - Da erro com o os LED. Pois na verdade, o ESP não tem pino livre
  // Serial.println("Starting SD Card");
  // if(!SD_MMC.begin()){
  //   Serial.println("SD Card Mount Failed");
  //   Erro = true;
  //   return;
  // }
  // uint8_t cardType = SD_MMC.cardType();
  // if(cardType == CARD_NONE){
  //   Serial.println("No SD Card attached");
  //   Erro = true;
  //   return;
  // }


  //OTA
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.setHostname("ESP32OTA-CAM");
  ArduinoOTA.setPassword("admin");
  ArduinoOTA.begin();



  if(Erro)
    digitalWrite(LED_PIN_R, HIGH);
  else
    digitalWrite(LED_PIN_G, HIGH);
}

void loop() {
  // if(Erro){
  //   Serial.println("An error occurred. Restarting ESP...");
  //   digitalWrite(LED_PIN, HIGH);
  //   delay(1000);
  //   ESP.restart();
  // }

  ArduinoOTA.handle();
  server.handleClient();
}
