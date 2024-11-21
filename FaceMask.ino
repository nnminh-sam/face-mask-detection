// * Wifi setup
const char* ssid       = "Noob";  
const char* password   = "leewang4u";
const char* apssid     = "esp32-cam";
const char* appassword = "12345678";

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "esp_camera.h"         
#include "soc/soc.h"           
#include "soc/rtc_cntl_reg.h" 
#include "UniversalTelegramBot.h"
#include "ArduinoJson.h"
#include <Arduino.h>
#include "index.h"

// * Define interation variables from the Telegram bot
String Feedback = "";
String Command = "", cmd = "", P1="", P2="", P3="", P4="",P5="", P6="", P7="", P8="", P9="";

byte ReceiveState=0,cmdState=1,strState=1,questionState=0,equalstate=0,semicolonState=0;

// * Telegram bot configuration
String BOTtoken = "7860451504:AAGs3BENbiF61MqDpnY5YRDbvnOtlwwXOeg";
String CHAT_ID = "5063928736";

// * Flash control
bool flashState = LOW;
int botRequestDelay = 1000;

unsigned long lastTimeBotRan;

bool sendPhoto = false;
WiFiClientSecure clientTCP;
UniversalTelegramBot bot(BOTtoken, clientTCP);

// * Define pins for camera GPIO
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_LED_PIN      4

// * Creating a Wifi server listening for incoming connection on port 80 (HTTP protocol)
WiFiServer server(80);

void executeCommand() {
  if (cmd != "getstill") {
    Serial.println("cmd= " + cmd + " ,P1= " + P1 + " ,P2= " + P2 + " ,P3= " + P3 + " ,P4= " + P4 + " ,P5= " + P5 + " ,P6= " + P6 + " ,P7= " + P7 + " ,P8= " + P8 + " ,P9= " + P9);
    Serial.println("");
  }
  if (cmd == "your cmd") {
  }
  else if (cmd == "ip") {
    Feedback = "AP IP: " + WiFi.softAPIP().toString();    
    Feedback += ", ";
    Feedback += "STA IP: " + WiFi.localIP().toString();
  } else if (cmd == "mac") {
    Feedback="STA MAC: " + WiFi.macAddress();
  }  
  else if (cmd == "resetwifi") {
    WiFi.begin(P1.c_str(), P2.c_str());
    Serial.print("Connecting to ");
    Serial.println(P1);

    long int StartTime = millis();
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      if ((StartTime + 5000) < millis()) {
        break;
      }
    }

    Serial.println("");
    Serial.println("STAIP: "+WiFi.localIP().toString());
    Feedback="STAIP: "+WiFi.localIP().toString();
  } else if (cmd == "restart") {
    ESP.restart();
  } else if (cmd == "digitalwrite") {
    ledcDetach(P1.toInt());
    pinMode(P1.toInt(), OUTPUT);
    digitalWrite(P1.toInt(), P2.toInt());
  } else if (cmd == "analogwrite") {
    if (P1 = "4") {
      ledcAttach(4, 5000, 8);
      ledcWrite(4,P2.toInt());  
    } else {
      ledcAttach(P1.toInt(), 5000, 8);
      ledcWrite(5,P2.toInt());
    }
  } else if (cmd == "flash") {
    ledcAttach(4, 5000, 8);  // Pin, frequency, resolution, thay thế cho ledcSetup và ledcAttachPin cũ
    int val = P1.toInt();     // Giá trị PWM được truyền vào
    ledcWrite(4, val);
  } else if (cmd == "framesize") { 
    sensor_t* s = esp_camera_sensor_get();  
    if (P1=="0")
      s->set_framesize(s, FRAMESIZE_QQVGA);
    else if (P1=="3")
      s->set_framesize(s, FRAMESIZE_HQVGA);
    else if (P1=="4")
      s->set_framesize(s, FRAMESIZE_QVGA);
    else if (P1=="5")
      s->set_framesize(s, FRAMESIZE_CIF);
    else if (P1=="6")
      s->set_framesize(s, FRAMESIZE_VGA);  
    else if (P1=="7")
      s->set_framesize(s, FRAMESIZE_SVGA);
    else if (P1=="8")
      s->set_framesize(s, FRAMESIZE_XGA);
    else if (P1=="9")
      s->set_framesize(s, FRAMESIZE_SXGA);
    else if (P1=="10")
      s->set_framesize(s, FRAMESIZE_UXGA);           
    else 
      s->set_framesize(s, FRAMESIZE_QVGA);     
  } else if (cmd == "quality") { 
    sensor_t* s = esp_camera_sensor_get();
    int val = P1.toInt(); 
    s->set_quality(s, val);
  } else if (cmd == "contrast") {
    sensor_t* s = esp_camera_sensor_get();
    int val = P1.toInt(); 
    s->set_contrast(s, val);
  } else if (cmd == "brightness") {
    sensor_t * s = esp_camera_sensor_get();
    int val = P1.toInt();  
    s->set_brightness(s, val);  
  } else if (cmd == "serial") { 
    if (P1 == "Mask") {
        String str = "Deo khau trang an toan\n";
        sendPhotoTelegram();
        bot.sendMessage(CHAT_ID, str, "");
        sendPhoto = false;
    } else if (P1 == "No%20mask") {
      String str = "Khong deo khau trang\n";
      sendPhotoTelegram();
      bot.sendMessage(CHAT_ID, str, "");
      sendPhoto = false;
    } else {
      String str = "Khong co nguoi\n";
      bot.sendMessage(CHAT_ID, str, "");
      sendPhoto = false;
    }
    if (P1 != "" & P1 != "stop") Serial.println(P1);
    if (P2 != "" & P2 != "stop") Serial.println(P2);
    Serial.println();
  } else {
    Feedback="Command is not defined.";
  } if (Feedback == "") Feedback = Command;  
}

void setup() {
  // * Disabling the brownout detector on the ESP32.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // * Setting up Serial communication
  Serial.begin(115200);
  Serial.setDebugOutput(true);  
  Serial.println();

  // * Configuring camera
  camera_config_t cameraConfig;
  cameraConfig.ledc_channel = LEDC_CHANNEL_0;
  cameraConfig.ledc_timer = LEDC_TIMER_0;
  cameraConfig.pin_d0 = Y2_GPIO_NUM;
  cameraConfig.pin_d1 = Y3_GPIO_NUM;
  cameraConfig.pin_d2 = Y4_GPIO_NUM;
  cameraConfig.pin_d3 = Y5_GPIO_NUM;
  cameraConfig.pin_d4 = Y6_GPIO_NUM;
  cameraConfig.pin_d5 = Y7_GPIO_NUM;
  cameraConfig.pin_d6 = Y8_GPIO_NUM;
  cameraConfig.pin_d7 = Y9_GPIO_NUM;
  cameraConfig.pin_xclk = XCLK_GPIO_NUM;
  cameraConfig.pin_pclk = PCLK_GPIO_NUM;
  cameraConfig.pin_vsync = VSYNC_GPIO_NUM;
  cameraConfig.pin_href = HREF_GPIO_NUM;
  cameraConfig.pin_sscb_sda = SIOD_GPIO_NUM;
  cameraConfig.pin_sscb_scl = SIOC_GPIO_NUM;
  cameraConfig.pin_pwdn = PWDN_GPIO_NUM;
  cameraConfig.pin_reset = RESET_GPIO_NUM;
  cameraConfig.xclk_freq_hz = 20000000;
  cameraConfig.pixel_format = PIXFORMAT_JPEG;

  // * Configuring camera base on the available RAM settings
  /**
    * The if statement is checking PSRAM is found on the ESP32 board?
    * PSRAM is an external memory module that provides additional RAM to the ESP32.
   */
  if (psramFound()) {
    cameraConfig.frame_size = FRAMESIZE_UXGA; // * Configuring camera resolution (1600x1200)
    // * compression quality of the captured images
    // * Lower values (e.g., 0-10): Higher image quality but larger file size.
    // * Higher values (e.g., 40-63): Lower image quality but smaller file size.
    cameraConfig.jpeg_quality = 10;
    // * The number of frame buffers to use.
    // * The ESP32 can capture a frame into one buffer while processing or transmitting the previous frame from the other buffer. This improves performance and reduces latency.
    cameraConfig.fb_count = 2;
  } else {
    cameraConfig.frame_size = FRAMESIZE_SVGA; // * Configuring camera resolution (800x600)
    // * compression quality of the captured images
    // * Lower values (e.g., 0-10): Higher image quality but larger file size.
    // * Higher values (e.g., 40-63): Lower image quality but smaller file size.
    cameraConfig.jpeg_quality = 12;
    // * The number of frame buffers to use.
    // * The ESP32 can capture a frame into one buffer while processing or transmitting the previous frame from the other buffer. This improves performance and reduces latency.
    cameraConfig.fb_count = 1;
  }
  
  // * Initialize camera
  esp_err_t err = esp_camera_init(&cameraConfig);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  // * Optimizing camera for start up
  // * Droping down frame size for higher initial frame rate
  sensor_t* s = esp_camera_sensor_get();
  // * | UXGA | SXGA | XGA | SVGA | VGA | CIF | QVGA | HQVGA | QQVGA |
  // * QVGA (320x240)
  s->set_framesize(s, FRAMESIZE_QVGA);

  // * Configuring PWM (Pulse Width Modulation) for an LED or a similar component connected to the ESP32.
  ledcAttach(4, 5000, 8);

  // * Setting up Wifi modes: Configures the ESP32 to operate in both Station mode (STA) and Access Point mode (AP) simultaneously.
  WiFi.mode(WIFI_AP_STA);

  WiFi.begin(ssid, password);

  // * Add root certificate for api.telegram.org
  clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT);

  delay(1000);
  Serial.println("");
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // * Connecting to wifi in 10 seconds
  long int startTime = millis(); // * Get current time in miliseconds as startTime
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if ((startTime + 10000) < millis()) {
      break;
    }
  } 

  // * Configuring Access Point (AP)
  if (WiFi.status() == WL_CONNECTED) {  
    WiFi.softAP((WiFi.localIP().toString() + "_" + (String)apssid).c_str(), appassword);
    Serial.println("");
    Serial.println("STAIP address: ");
    Serial.println(WiFi.localIP());

    // * Blink LED 5 times for success connection
    for (int i = 0; i < 5; i++) {
      ledcWrite(4,10);
      delay(200);
      ledcWrite(4,0);
      delay(200);    
    }
  } else {
    WiFi.softAP((WiFi.softAPIP().toString() + "_" + (String)apssid).c_str(), appassword);
    Serial.println("Failed to connect to Wifi");

    // * Blink LED 2 times for failed connection
    for (int i = 0; i < 2; i++) {   
      ledcWrite(4,10);
      delay(1000);
      ledcWrite(4,0);
      delay(1000);    
    }
  }     

  // * Print Access Point IP Address
  Serial.println("");
  Serial.println("APIP address: ");
  Serial.println(WiFi.softAPIP());    

  // * Configures GPIO pin 4 as an output
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  // * Start Wifi server for handling incoming HTTP requests
  server.begin();      
}

void handleNewMessages(int numNewMessages) {
  Serial.print("Handle New Messages: ");
  Serial.println(numNewMessages);

  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID){
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }

    // * Print the received message
    String text = bot.messages[i].text;
    Serial.println(text);
    String from_name = bot.messages[i].from_name;

    if (text == "/start") {
      String welcome = "Welcome , " + from_name + "\n";
      welcome += "Use the following commands to interact with the ESP32-CAM \n";
      welcome += "/photo : takes a new photo\n";
      welcome += "/flash : toggles flash LED \n";
      bot.sendMessage(CHAT_ID, welcome, "");
    }

    if (text == "/flash") {
      flashState = !flashState;
      digitalWrite(FLASH_LED_PIN, flashState);
      Serial.println("Change flash LED state");
    }

    if (text == "/photo") {
      sendPhoto = true;
      Serial.println("New photo request");
    }
  }
}

String sendPhotoTelegram() {
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";

  // * Capturing image from camera
  camera_fb_t* frameBuffer = NULL;
  frameBuffer = esp_camera_fb_get(); // * Capture a frame buffer
  esp_camera_fb_return(frameBuffer); // * dispose the buffered image
  frameBuffer = NULL;
  frameBuffer = esp_camera_fb_get(); // * Capture a fresh frame

  // * Restart ESP32 board when second capture failed
  if (!frameBuffer) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
    return "Camera capture failed";
  }

  // * Telegram connection
  Serial.println("Connect to " + String(myDomain));
  if (clientTCP.connect(myDomain, 443)) { // * Success connection to Telegram
    Serial.println("Connection successful");

    // * Creating HTTP POST Request for connecting to Telegram
    String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + CHAT_ID + "\r\n--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--RandomNerdTutorials--\r\n";
    size_t imageLen = frameBuffer->len;
    size_t extraLen = head.length() + tail.length();
    size_t totalLen = imageLen + extraLen;
    clientTCP.println("POST /bot" + BOTtoken + "/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    clientTCP.println();
    clientTCP.print(head);

    // * Send image data in chunk of 1024 bytes to server
    uint8_t* fbBuf = frameBuffer->buf;
    size_t fbLen = frameBuffer->len;
    for (size_t n = 0; n < fbLen; n = n + 1024) {
      if (n + 1024 < fbLen) {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen % 1024 > 0) {
        size_t remainder = fbLen % 1024;
        clientTCP.write(fbBuf, remainder);
      }
    }  
    clientTCP.print(tail);
    esp_camera_fb_return(frameBuffer);

    // * Handle server response
    int waitTime = 10000; // * response timeout in 10 seconds
    long startTimer = millis();
    boolean state = false;
    while ((startTimer + waitTime) > millis()){
      Serial.print(".");
      delay(100);      
      while (clientTCP.available()) {
        char c = clientTCP.read();
        if (state == true) getBody += String(c);        
        if (c == '\n') {
          if (getAll.length() == 0) state = true; 
          getAll = "";
        } 
        else if (c != '\r')
          getAll += String(c);
        startTimer = millis();
      }
      if (getBody.length() > 0) break;
    }
    clientTCP.stop();
    Serial.println(getBody);
  }
  else {
    getBody = "Connected to api.telegram.org failed.";
    Serial.println("Connected to api.telegram.org failed.");
  }
  return getBody;
}

void loop() {
  // * Send photo to Telegram
  if (sendPhoto) {
    Serial.println("Preparing photo");
    sendPhotoTelegram(); 
    sendPhoto = false; 
  }

  // * Handle message from Telegram bot
  if (millis() > lastTimeBotRan + botRequestDelay)  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages) {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }
  
  Feedback = "";
  Command = "";
  cmd = "";
  P1 = "";
  P2 = "";
  P3 = "";
  P4 = "";
  P5 = "";
  P6 = "";
  P7 = "";
  P8 = "";
  P9 = "";
  ReceiveState = 0, cmdState = 1, strState = 1, questionState = 0, equalstate = 0, semicolonState =0;

  // * Get Wifi client availability
  WiFiClient client = server.available();
  if (client) { 
    String currentLine = "";

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();             

        getCommand(c);

        if (c == '\n') {
          if (currentLine.length() == 0) {    
            // * Capture and send image to HTTP Client
            if (cmd == "getstill") {
              camera_fb_t * fb = NULL;
              fb = esp_camera_fb_get();
              if (!fb) {
                Serial.println("Camera capture failed");
                delay(1000);
                ESP.restart();
              }

              // * Create POST HTTP Request
              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Origin: *");              
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: image/jpeg");
              client.println("Content-Disposition: form-data; name=\"imageFile\"; filename=\"picture.jpg\""); 
              client.println("Content-Length: " + String(fb->len));             
              client.println("Connection: close");
              client.println();

              // * Write data in chunk of 1024 bytes
              uint8_t* fbBuf = fb->buf;
              size_t fbLen = fb->len;
              for (size_t n = 0; n < fbLen; n = n + 1024) {
                if (n + 1024 < fbLen) {
                  client.write(fbBuf, 1024);
                  fbBuf += 1024;
                } else if (fbLen % 1024 > 0) {
                  size_t remainder = fbLen % 1024;
                  client.write(fbBuf, remainder);
                }
              }  

              esp_camera_fb_return(fb);

              pinMode(4, OUTPUT);
              digitalWrite(4, LOW);               
            } else {
              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: text/html; charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              String Data = "";
              if (cmd != "") {
                Data = Feedback;
              } else {
                Data = String((const char *)INDEX_HTML);
              }

              int Index;
              for (Index = 0; Index < Data.length(); Index = Index + 1000) {
                client.print(Data.substring(Index, Index + 1000));
              }           

              client.println();
            }

            Feedback="";
            break;
          } else {
            currentLine = "";
          }
        } 
        else if (c != '\r') {
          currentLine += c;
        }

        if ((currentLine.indexOf("/?") != -1) && (currentLine.indexOf(" HTTP") != -1)) {
          if (Command.indexOf("stop") != -1) {  
            client.println();
            client.println();
            client.stop();
          }

          currentLine = "";
          Feedback = "";
          executeCommand();
        }
      }
    }
    delay(1);
    client.stop();
  }
}

void getCommand(char c) {
  if (c == '?') ReceiveState = 1;

  if ((c == ' ') || (c == '\r') || (c == '\n')) ReceiveState = 0;
  
  if (ReceiveState == 1) {
    Command = Command + String(c);
    
    if (c == '=') cmdState = 0;
    if (c == ';') strState++;
  
    if ((cmdState == 1) && ((c != '?')||(questionState == 1))) cmd = cmd + String(c);
    if ((cmdState == 0) && (strState == 1) && (( c != '=') || (equalstate == 1))) P1 = P1 + String(c);
    if ((cmdState == 0) && (strState == 2) && (c != ';')) P2 = P2 + String(c);
    if ((cmdState == 0) && (strState == 3) && (c != ';')) P3 = P3 + String(c);
    if ((cmdState == 0) && (strState == 4) && (c != ';')) P4 = P4 + String(c);
    if ((cmdState == 0) && (strState == 5) && (c != ';')) P5 = P5 + String(c);
    if ((cmdState == 0) && (strState == 6) && (c != ';')) P6 = P6 + String(c);
    if ((cmdState == 0) && (strState == 7) && (c != ';')) P7 = P7 + String(c);
    if ((cmdState == 0) && (strState == 8) && (c != ';')) P8 = P8 + String(c);
    if ((cmdState == 0) && (strState >= 9) && ((c != ';') || (semicolonState == 1))) P9 = P9 + String(c);
    
    if (c == '?') questionState = 1;
    if (c == '=') equalstate = 1;
    if ((strState >= 9) && (c == ';')) semicolonState = 1;
  }
}