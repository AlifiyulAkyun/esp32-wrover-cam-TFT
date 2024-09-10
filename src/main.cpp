#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "esp_camera.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include <TFT_eFEX.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <WebSocketsClient.h>
#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"
#include "dl_tool.hpp"
#include "fb_gfx.h"

const bool debug = false;
// const bool debug = true;

WiFiClient espClient;
PubSubClient client(espClient);

#define FACE_COLOR_GREEN 0x0000FF00

#define TWO_STAGE 1 /*<! 1: detect by two-stage which is more accurate but slower(with keypoints). */
                    /*<! 0: detect by one-stage which is less accurate but faster(without keypoints). */

#define TEST_CAPTURE 0

TFT_eSPI tft = TFT_eSPI();
TFT_eFEX fex = TFT_eFEX(&tft);

camera_fb_t *fb = NULL;
camera_config_t cameraConfig;

WebSocketsClient webSocket;
// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

const char *ssid = "IOT_SERVER";
const char *password = "1234567889";
const char *hostWs = "192.168.43.142";
const char *mqtt_server = "broker.sinaungoding.com";
const uint16_t portWs = 8765;
const int mqttPort = 1883;
const char *mqtt_username = "uwais";
const char *mqtt_password = "uw415_4Lqarn1";
const char *mqtt_clientId = "esp32-client";
// const char *ssid = "JTI-POLINEMA";
// const char *password = "jtifast!";
// const char *hostWs = "192.168.74.45";
boolean isImageProcessing = true;

unsigned long previousMilis = 0;
const long interval = 10000; // internval

boolean initCamera(camera_config_t config);
void initWiFi(char *ssid, char *password);
void initMQTT();
void reconnect();
void sendMQTTMessage(const char *topic, const char *message);
void callback(char *topic, byte *payload, unsigned int length);
void onWebsocketEvent(WStype_t type, uint8_t *payload, size_t length);
static void draw_face_boxes(fb_data_t *fb, std::list<dl::detect::result_t> *results);
void printTextTft(uint32_t colorScreen, uint32_t colorText, uint8_t sizeText, int16_t x, int16_t y, String text);
void printTextTft(uint32_t colorText, uint8_t sizeText, int16_t x, int16_t y, String text);

void setup()
{
  Serial.begin(115200);

  tft.begin();
  tft.setRotation(3); // 0 & 2 Portrait. 1 & 3 landscape
  printTextTft(TFT_BLACK, TFT_RED, 2, 0, 0, "Hello TFT");
  delay(1000);
  initWiFi((char *)ssid, (char *)password);
  delay(3000);

  webSocket.begin(hostWs, portWs);
  webSocket.onEvent(onWebsocketEvent);
  webSocket.setReconnectInterval(5000);
  webSocket.sendTXT(WiFi.localIP().toString().c_str());

  printTextTft(TFT_WHITE, TFT_BLUE, 1, 0, 0, "Websocket connecting ...");
  printTextTft(TFT_PURPLE, 1, 0, 10, String(hostWs));
  printTextTft(TFT_PURPLE, 1, 0, 20, String(webSocket.isConnected() ? "Succeed" : "Failed"));
  log_d("%s", webSocket.isConnected() ? "websocket connected" : "websocket not connected");
  delay(3000);
  if (!initCamera(cameraConfig))
  {
    log_i("Restarting esp....");
    esp_restart();
  }

  // Initialize a NTPClient to get time
  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  timeClient.setTimeOffset(25200); // GMT + 7
                                   // Initialize MQTT
  initMQTT();
}

size_t out_len = 0, out_width = 0, out_height = 0;
dl::tool::Latency latency;
#if TWO_STAGE
HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
#else
HumanFaceDetectMSR01 s1(0.3F, 0.5F, 10, 0.2F);
#endif
uint8_t *out_buf;
bool s;
bool detected = false;
esp_err_t res = ESP_OK;
size_t _jpg_buf_len = 0, _jpg_buf_result_len = 0;
uint8_t *_jpg_buf = NULL;
uint8_t *_jpg_buf_result = NULL;

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi not connected");
    initWiFi((char *)ssid, (char *)password);
  }
  // Ensure MQTT connection is alive
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  // Get current time from NTP server
  while (!timeClient.update())
  {
    timeClient.forceUpdate();
  }
  // Example: send MQTT message every 10 seconds
  unsigned long currentMillis = millis();
  static unsigned long previousMillis = 0;
  const long interval = 10000; // 10 seconds

  if (currentMillis - previousMillis >= interval)
  {
    sendMQTTMessage("esp32/output", "Hello from ESP32");
    previousMillis = currentMillis;
  }

  // Ensure MQTT client is connected and handle reconnecting if necessary
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  res = ESP_OK;
  fb = NULL;
  fb = esp_camera_fb_get();
  detected = false;
  isImageProcessing = false;
  unsigned long currentMilis = millis();

  if (!fb)
  {
    Serial.println("Camera capture failed");
    return;
  }

  // fex.drawJpg((uint8_t *)fb->buf, fb->len, 0, 0);

  _jpg_buf_len = fb->len;
  _jpg_buf = fb->buf;

  out_len = fb->width * fb->height * 3;
  out_width = fb->width;
  out_height = fb->height;
  out_buf = (uint8_t *)malloc(out_len);
  if (!out_buf)
  {
    log_e("out_buf malloc failed");
    res = ESP_FAIL;
  }
  else
  {
    log_d("To rgb888");
    s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
    log_d("rgb888 finished");
    esp_camera_fb_return(fb);
    fb = NULL;
    if (!s)
    {
      free(out_buf);
      log_e("To rgb888 failed");
      res = ESP_FAIL;
    }
    else
    {
      log_d("fb_data_t");
      fb_data_t rfb;
      rfb.width = out_width;
      rfb.height = out_height;
      rfb.data = out_buf;
      rfb.bytes_per_pixel = 3;
      rfb.format = FB_BGR888;

      // inference
      latency.start();
#if TWO_STAGE
      std::list<dl::detect::result_t> &candidates = s1.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3});
      std::list<dl::detect::result_t> &results = s2.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3}, candidates);
#else
      std::list<dl::detect::result_t> &results = s1.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3});
#endif
      latency.end();
      latency.print("Inference latency");
      if (results.size() > 0)
      {
        int i = 0;
        for (std::list<dl::detect::result_t>::iterator prediction = results.begin(); prediction != results.end(); prediction++, i++)
        {
          log_d("[%d] score: %f, box: [%d, %d, %d, %d]\n", i, prediction->score, prediction->box[0], prediction->box[1], prediction->box[2], prediction->box[3]);

#if TWO_STAGE
          log_d("    left eye: (%3d, %3d), ", prediction->keypoint[0], prediction->keypoint[1]);
          log_d("right eye: (%3d, %3d)\n", prediction->keypoint[6], prediction->keypoint[7]);
          log_d("    nose: (%3d, %3d)\n", prediction->keypoint[4], prediction->keypoint[5]);
          log_d("    mouth left: (%3d, %3d), ", prediction->keypoint[2], prediction->keypoint[3]);
          log_d("mouth right: (%3d, %3d)\n\n", prediction->keypoint[8], prediction->keypoint[9]);
#endif
        }

        s = fmt2jpg(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, &_jpg_buf_result, &_jpg_buf_result_len);
        if (!s)
        {
          log_e("fmt2jpg sent failed");
          res = ESP_FAIL;
        }
        detected = true;
        draw_face_boxes(&rfb, &results);
        log_i("Face detected");
      }
      s = fmt2jpg(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len);
      free(out_buf);
      if (!s)
      {
        log_e("fmt2jpg draw failed");
        res = ESP_FAIL;
      }
    }
  }

  if (res == ESP_OK)
  {
    log_d("draw to screen");
    fex.drawJpg((uint8_t *)_jpg_buf, _jpg_buf_len, 0, 0);
  }
  log_d("%s", webSocket.isConnected() ? "True" : "False");
  log_d("%s", isImageProcessing ? "True" : "False");
  log_d("%s", detected ? "True" : "False");
  log_d("%d", ESP_OK);
  String formattedDate = timeClient.getFormattedDate();
  // Extract date
  int splitT = formattedDate.indexOf("T");
  String dayStamp = formattedDate.substring(0, splitT);
  // Serial.println(dayStamp);
  log_d("datetime: %s", dayStamp);
  // Extract time
  String timeStamp = formattedDate.substring(splitT + 1, formattedDate.length() - 1);
  // Serial.println(timeStamp);
  log_d("datetime: %s", timeStamp);
  String rs = dayStamp;
  rs += " ";
  rs += timeStamp;
  Serial.println(rs);
  if (detected)
  {
    // sending image to websocket
    log_d("%s %s", webSocket.isConnected() ? "websocket connected" : "websocket not connected", isImageProcessing ? "True" : "False");
    if (webSocket.isConnected() && !isImageProcessing && currentMilis - previousMilis >= interval)
    {
      webSocket.sendBIN((uint8_t *)_jpg_buf_result, _jpg_buf_result_len);
      isImageProcessing = true;
      log_d("image sent to server through websocket");
      previousMilis = currentMilis;
    }
    else
    {
      log_d("not send to server");
    }

    free(_jpg_buf_result);
    _jpg_buf_result = NULL;
  }

#if TEST_CAPTURE
  // testing purpose send data image
  if (currentMilis - previousMilis >= interval)
  {
    log_i("send data to server [testing purpose]");
    if (webSocket.isConnected())
    {
      previousMilis = currentMilis;
      webSocket.sendBIN((uint8_t *)_jpg_buf, _jpg_buf_len);
    }
  }
#endif

  if (fb)
  {
    esp_camera_fb_return(fb);
    fb = NULL;
    _jpg_buf = NULL;
  }
  else if (_jpg_buf)
  {
    free(_jpg_buf);
    _jpg_buf = NULL;
  }
  else
  {
    log_e("Send frame failed");
  }

  webSocket.loop();
}

void initWiFi(char *ssid, char *password)
{
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }
  Serial.printf("\nConnected to WiFi\n");
}

boolean initCamera(camera_config_t config)
{
  // Configure GPIO pin assignments for camera interface
  config.pin_pwdn = -1;     // Power-down pin (not used)
  config.pin_reset = -1;    // Reset pin (not used)
  config.pin_xclk = 21;     // External clock pin (CAM_XCLK)
  config.pin_sscb_sda = 26; // SCCB data pin (CAM_SIOD)
  config.pin_sscb_scl = 27; // SCCB clock pin (CAM_SIOC)
  config.pin_d7 = 35;       // Data line D7 (CAM_Y9)
  config.pin_d6 = 34;       // Data line D6 (CAM_Y8)
  config.pin_d5 = 39;       // Data line D5 (CAM_Y7)
  config.pin_d4 = 36;       // Data line D4 (CAM_Y6)
  config.pin_d3 = 19;       // Data line D3 (CAM_Y5)
  config.pin_d2 = 18;       // Data line D2 (CAM_Y4)
  config.pin_d1 = 5;        // Data line D1 (CAM_Y3)
  config.pin_d0 = 4;        // Data line D0 (CAM_Y2)

  config.pin_vsync = 25; // VSYNC pin (CAM_VSYNC)
  config.pin_href = 23;  // HREF pin (CAM_HREF)
  config.pin_pclk = 22;  // Pixel clock pin (CAM_PCLK)

  config.xclk_freq_hz = 20000000;       // External clock frequency (20 MHz)
  config.pixel_format = PIXFORMAT_JPEG; // Pixel format (JPEG)
  // config.pixel_format = PIXFORMAT_RGB565; // Pixel format (JPEG)

  config.frame_size = FRAMESIZE_240X240;
  config.jpeg_quality = 10;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM; // Frame buffer location
  config.grab_mode = CAMERA_GRAB_LATEST;
  // Initialize camera with the configured parameters
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    log_e("Camera initialization failed!");
    log_e("Error code: 0x%x \n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1);

  return true;
}
void initMQTT()
{
  client.setServer(mqtt_server, mqttPort);
  client.setCallback(callback);
  reconnect();
}
void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(mqtt_clientId, mqtt_username, mqtt_password))
    {
      Serial.println("connected");
      client.subscribe("esp32/input"); // Subscribe to MQTT topic
      client.subscribe("esp32/json");  // Tambahkan topik baru untuk menerima pesan JSON
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void sendMQTTMessage(const char *topic, const char *message)
{
  if (!client.connected())
  {
    reconnect();
  }
  client.publish(topic, message); // Publish message to MQTT topic
}
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

  if (String(topic) == "esp32/json") // Periksa apakah pesan datang dari topik baru
  {
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      return;
    }

    const char *status = doc["status"];
    const char *label = doc["label"];
    const char *confident = doc["confident"];
    const char *datetime = doc["datetime"];

    Serial.printf("Status: %s\n", status);
    Serial.printf("Recognized Label: %s\n", label);
    Serial.printf("Confidence: %s\n", confident);
    Serial.printf("Datetime: %s\n", datetime);

    // Implementasikan logika yang diinginkan untuk pesan JSON yang diterima
  }
}

// Fungsi untuk membuat objek JSON dengan waktu dari NTP client
void sendJsonWithDateTime(WebSocketsClient &webSocket, const char *status, const char *label, const char *confident)
{
  DynamicJsonDocument doc(1024);
  doc["status"] = status;
  doc["label"] = label;
  doc["confident"] = confident;

  // Mengambil waktu dari NTP client
  String formattedDate = timeClient.getFormattedDate();
  int splitT = formattedDate.indexOf("T");
  String dayStamp = formattedDate.substring(0, splitT);
  String timeStamp = formattedDate.substring(splitT + 1, formattedDate.length() - 1);
  String datetime = dayStamp + " " + timeStamp;

  doc["datetime"] = datetime;
  // Mencetak waktu di bawah confident
  Serial.printf("Datetime: %s\n", datetime.c_str());

  String output;
  serializeJson(doc, output);
  webSocket.sendTXT(output);

  // Mengirimkan data ke MQTT
  sendMQTTMessage("esp32/json_output", output.c_str());
}

void onWebsocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_DISCONNECTED:
    Serial.println("WS Disconnected");
    break;
  case WStype_CONNECTED:
    Serial.println("WS Connected");
    webSocket.sendTXT("Hello from ESP32");
    break;
  case WStype_TEXT:
  {
    Serial.println("Message Received from Server");
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, payload);
    if (error)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      return;
    }
    const char *status = doc["status"];
    const char *label = doc["label"];
    const char *confident = doc["confident"];

    Serial.printf("Status: %s\n", status);
    Serial.printf("Recognized Label: %s\n", label);
    Serial.printf("Confidence: %s\n", confident);

    // Mengirim objek JSON dengan waktu dari NTP client ke MQTT
    String jsonMessage;
    serializeJson(doc, jsonMessage);
    // sendMQTTMessage("esp32/json", jsonMessage.c_str());

    // Mengirim objek JSON dengan waktu dari NTP client melalui WebSocket
    sendJsonWithDateTime(webSocket, status, label, confident);

    break;
  }
  case WStype_ERROR:
    Serial.println("WS Error");
    break;
  default:
    break;
  }
}

static void draw_face_boxes(fb_data_t *fb, std::list<dl::detect::result_t> *results)
{
  int x, y, w, h;
  uint32_t color = FACE_COLOR_GREEN;
  if (fb->bytes_per_pixel == 2)
  {
    // color = ((color >> 8) & 0xF800) | ((color >> 3) & 0x07E0) | (color & 0x001F);
    color = ((color >> 16) & 0x001F) | ((color >> 3) & 0x07E0) | ((color << 8) & 0xF800);
  }
  int i = 0;
  for (std::list<dl::detect::result_t>::iterator prediction = results->begin(); prediction != results->end(); prediction++, i++)
  {
    // rectangle box
    x = (int)prediction->box[0];
    y = (int)prediction->box[1];
    w = (int)prediction->box[2] - x + 1;
    h = (int)prediction->box[3] - y + 1;
    if ((x + w) > fb->width)
    {
      w = fb->width - x;
    }
    if ((y + h) > fb->height)
    {
      h = fb->height - y;
    }
    // fex.drawRect(x, y, w, h, TFT_GREEN);
    fb_gfx_drawFastHLine(fb, x, y, w, color);
    fb_gfx_drawFastHLine(fb, x, y + h - 1, w, color);
    fb_gfx_drawFastVLine(fb, x, y, h, color);
    fb_gfx_drawFastVLine(fb, x + w - 1, y, h, color);

#if TWO_STAGE
    // landmarks (left eye, mouth left, nose, right eye, mouth right)
    int x0, y0, j;
    for (j = 0; j < 10; j += 2)
    {
      x0 = (int)prediction->keypoint[j];
      y0 = (int)prediction->keypoint[j + 1];
      // fex.fillRect(x0, y0, 3, 3, TFT_GREEN);
      fb_gfx_fillRect(fb, x0, y0, 3, 3, color);
    }
#endif
  }
}

void printTextTft(uint32_t colorScreen, uint32_t colorText, uint8_t sizeText, int16_t x, int16_t y, String text)
{
  tft.fillScreen(colorScreen);
  tft.setCursor(x, y);
  tft.setTextColor(colorText);
  tft.setTextSize(sizeText);
  tft.println(text);
}

void printTextTft(uint32_t colorText, uint8_t sizeText, int16_t x, int16_t y, String text)
{
  tft.setCursor(x, y);
  tft.setTextColor(colorText);
  tft.setTextSize(sizeText);
  tft.println(text);
}