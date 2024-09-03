#include <ArduinoWebsockets.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_index.h"
#include "Arduino.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"
#include "UniversalTelegramBot.h"
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>

const char* ssid = "Pohon Belimbing";
const char* password = "klentang25";

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 2

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// Initialize Telegram BOT
String BOTtoken = "5584847809:AAHvNZBJrV6PG1hM2yrG6rAfLl0ugER2iio";  // your Bot Token (Get from Botfather)
// Use @myidbot to find out the chat ID of an individual or a group
// Also note that you need to click "start" on a bot before it can
// message you
String CHAT_ID = "680638640";

bool sendPhoto = false, PreparePhoto = false;

WiFiClientSecure clientTCP;
UniversalTelegramBot bot(BOTtoken, clientTCP);

using namespace websockets;
WebsocketsServer socket_server;

#define BuzzerPin 14
#define FlashPin 13
#define SelenoidPin 15
#define DoorSwitchPin 4
#define PushButtonPin 2
//#define FlashPin 4

camera_fb_t * fb = NULL;

long current_millis;
long last_detected_millis = 0;

void app_facenet_main();
void app_httpserver_init();

typedef struct
{
  uint8_t *image;
  box_array_t *net_boxes;
  dl_matrix3d_t *face_id;
} http_img_process_result;

static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();

face_id_name_list st_face_list;
static dl_matrix3du_t *aligned_face = NULL;

httpd_handle_t camera_httpd = NULL;

typedef enum
{
  START_STREAM,
  START_DETECT,
  SHOW_FACES,
  START_RECOGNITION,
  START_ENROLL,
  ENROLL_COMPLETE,
  DELETE_ALL,
} en_fsm_state;
en_fsm_state g_state;

typedef struct
{
  char enroll_name[ENROLL_NAME_LEN];
} httpd_resp_value;

httpd_resp_value st_name;
dl_matrix3du_t *image_matrix;
http_img_process_result out_res;

bool flashState = LOW;

int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

int buttonState;
int lastButtonState = LOW;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 100;
boolean buttonActive = false;
boolean longPressActive = false, shortPressActive = false;
long buttonTimer = 0;
long longPressTime = 2000;

#define TimeWaitDoor 10000
unsigned long StartWait = 0;
boolean DoorOpen = false, DoorSwitch = false;

void handleNewMessages(int numNewMessages) {
  Serial.print("Handle New Messages: ");
  Serial.println(numNewMessages);

  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID){
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }
    
    // Print the received message
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
      digitalWrite(FlashPin, flashState);
      Serial.println("Change flash LED state");
    }
    if (text == "/photo") {
      PreparePhoto = true;
//      sendPhoto = true;
      Serial.println("New photo request");
    }
  }
}

String sendPhotoTelegram(bool state, String namee) {
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";

  socket_server.~WebsocketsServer();

  Serial.println("Connect to " + String(myDomain));

  if (clientTCP.connect(myDomain, 443)) {
    Serial.println("Connection successful");
    
    String head = "--SecurityAccessRoom\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + CHAT_ID + "\r\n--SecurityAccessRoom\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--SecurityAccessRoom--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;
  
    clientTCP.println("POST /bot"+BOTtoken+"/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=SecurityAccessRoom");
    clientTCP.println();
    clientTCP.print(head);

    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0;n<fbLen;n=n+1024) {
      if (n+1024<fbLen) {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        clientTCP.write(fbBuf, remainder);
      }
    }  
    
    clientTCP.print(tail);
    
    esp_camera_fb_return(fb);
    
    int waitTime = 10000;   // timeout 10 seconds
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + waitTime) > millis()){
      Serial.print(".");
      delay(100);      
      while (clientTCP.available()) {
        char c = clientTCP.read();
        if (state==true) getBody += String(c);        
        if (c == '\n') {
          if (getAll.length()==0) state=true; 
          getAll = "";
        } 
        else if (c != '\r')
          getAll += String(c);
        startTimer = millis();
      }
      if (getBody.length()>0) break;
    }
    clientTCP.stop();
    Serial.println("getBody");
    Serial.println(getBody);

    String Access;
    if(state) {
      Access = "Room Accessing by " + namee + "\n";
//      sprintf(Access, "Room Accessing by %s\n", namee);
    }
    else {
      Access = "Somebody Try to Accessing Room\n";      
    }
    bot.sendMessage(CHAT_ID, Access, "");

  }
  else {
    getBody="Connected to api.telegram.org failed.";
    Serial.println("Connected to api.telegram.org failed.");
  }
  return getBody;

  delay(1000);
  app_httpserver_init();
  app_facenet_main();
  socket_server.listen(82);  
}

void CekTombol() {
  int reading = digitalRead(PushButtonPin);
  if (reading != lastButtonState) {
    Serial.println("CekTombol");
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) {
        if (buttonActive == false) {
          buttonActive = true;
          buttonTimer = millis();
        }
        while(digitalRead(PushButtonPin) == HIGH) {
          Serial.println("Button High");
          if ((millis() - buttonTimer > longPressTime) && (longPressActive == false)) {
            Serial.println("Long Press");
            longPressActive = true;
            shortPressActive = false;
          }
          else {
            Serial.println("Short Press");
            longPressActive = false;
            shortPressActive = true;
          }
        }
      }
    }
  }
  lastButtonState = reading;
}

void OpenDoor() {
  digitalWrite(SelenoidPin, HIGH);
  StartWait = millis();
  DoorOpen = true;
  DoorSwitch = true;
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("Start");

// Set LED Flash as output
  pinMode(FlashPin, OUTPUT);
  pinMode(BuzzerPin, OUTPUT);
  pinMode(SelenoidPin, OUTPUT);
  pinMode(DoorSwitchPin, INPUT);
  pinMode(PushButtonPin, INPUT);
  
  digitalWrite(BuzzerPin, flashState);
  
  digitalWrite(FlashPin, HIGH); delay(1000);
  digitalWrite(FlashPin, LOW);

  digitalWrite(SelenoidPin, HIGH); delay(1000);
  digitalWrite(SelenoidPin, LOW);

  Serial.print("Door Switch State "); Serial.println(digitalRead(DoorSwitchPin));
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);  // UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
  
#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  WiFi.begin(ssid, password);
  clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
}

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}

httpd_uri_t index_uri = {
  .uri       = "/",
  .method    = HTTP_GET,
  .handler   = index_handler,
  .user_ctx  = NULL
};

void app_httpserver_init ()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  if (httpd_start(&camera_httpd, &config) == ESP_OK)
    Serial.println("httpd_start");
  {
    httpd_register_uri_handler(camera_httpd, &index_uri);
  }
}

void app_facenet_main()
{
  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
  read_face_id_from_flash_with_name(&st_face_list);
}

static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id)
{
  ESP_LOGD(TAG, "START ENROLLING");
  int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, st_name.enroll_name);
  ESP_LOGD(TAG, "Face ID %s Enrollment: Sample %d",
           st_name.enroll_name,
           ENROLL_CONFIRM_TIMES - left_sample_face);
  return left_sample_face;
}

static esp_err_t send_face_list(WebsocketsClient &client)
{
  client.send("delete_faces"); // tell browser to delete all faces
  face_id_node *head = st_face_list.head;
  char add_face[64];
  for (int i = 0; i < st_face_list.count; i++) // loop current faces
  {
    sprintf(add_face, "listface:%s", head->id_name);
    client.send(add_face); //send face to browser
    head = head->next;
  }
}

static esp_err_t delete_all_faces(WebsocketsClient &client)
{
  delete_face_all_in_flash_with_name(&st_face_list);
  client.send("delete_faces");
}

void handle_message(WebsocketsClient &client, WebsocketsMessage msg)
{
  if (msg.data() == "stream") {
    g_state = START_STREAM;
    client.send("STREAMING");
  }
  if (msg.data() == "detect") {
    g_state = START_DETECT;
    client.send("DETECTING");
  }
  if (msg.data().substring(0, 8) == "capture:") {
    g_state = START_ENROLL;
    char person[FACE_ID_SAVE_NUMBER * ENROLL_NAME_LEN] = {0,};
    msg.data().substring(8).toCharArray(person, sizeof(person));
    memcpy(st_name.enroll_name, person, strlen(person) + 1);
    client.send("CAPTURING");
  }
  if (msg.data() == "recognise") {
    g_state = START_RECOGNITION;
    client.send("RECOGNISING");
  }
  if (msg.data().substring(0, 7) == "remove:") {
    char person[ENROLL_NAME_LEN * FACE_ID_SAVE_NUMBER];
    msg.data().substring(7).toCharArray(person, sizeof(person));
    delete_face_id_in_flash_with_name(&st_face_list, person);
    send_face_list(client); // reset faces in the browser
  }
  if (msg.data() == "delete_all") {
    delete_all_faces(client);
  }
}

void Capture(http_img_process_result out_res)
{
  fb = esp_camera_fb_get();

  if (g_state == START_DETECT || g_state == START_ENROLL || g_state == START_RECOGNITION)
  {
    out_res.net_boxes = NULL;
    out_res.face_id = NULL;

    fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image);

    out_res.net_boxes = face_detect(image_matrix, &mtmn_config);

    if (out_res.net_boxes)
    {
      if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK)
      {
        out_res.face_id = get_face_id(aligned_face);
        last_detected_millis = millis();
        if (g_state == START_DETECT) {
          Serial.println("FACE DETECTED");
        }

        if (g_state == START_ENROLL)
        {
          int left_sample_face = do_enrollment(&st_face_list, out_res.face_id);
          char enrolling_message[64];
          sprintf(enrolling_message, "SAMPLE NUMBER %d FOR %s", ENROLL_CONFIRM_TIMES - left_sample_face, st_name.enroll_name);
          Serial.println(enrolling_message);
          if (left_sample_face == 0)
          {
            ESP_LOGI(TAG, "Enrolled Face ID: %s", st_face_list.tail->id_name);
            g_state = START_STREAM;
            char captured_message[64];
            sprintf(captured_message, "FACE CAPTURED FOR %s", st_face_list.tail->id_name);
            Serial.println(captured_message);
//            send_face_list(client);

          }
        }

        if (g_state == START_RECOGNITION  && (st_face_list.count > 0))
        {
          face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id);
          if (f)
          {
            char recognised_message[64];
            sprintf(recognised_message, "RECOGNISED %s", f->id_name);
//            client.send(recognised_message);
            Serial.println(recognised_message);
            sendPhotoTelegram(1, f->id_name);
            OpenDoor();
          }
          else
          {
        //    client.send("FACE NOT RECOGNISED");
            Serial.println("FACE NOT RECOGNISED");
            sendPhotoTelegram(0, "Unknown");
          }  
        }
        dl_matrix3d_free(out_res.face_id);
      }
    }
    else
    {
      if (g_state != START_DETECT) {
        Serial.println("NO FACE DETECTED");
      }
    }

    if (g_state == START_DETECT && millis() - last_detected_millis > 500) { // Detecting but no face detected
      Serial.println("DETECTING");
    }

  }

  esp_camera_fb_return(fb);
  fb = NULL;
}

void HandleCapture(WebsocketsClient &client, http_img_process_result out_res)
{
  face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id);
  if (f)
  {
    char recognised_message[64];
    sprintf(recognised_message, "RECOGNISED %s", f->id_name);
    client.send(recognised_message);
    Serial.println(recognised_message);
    sendPhotoTelegram(1, f->id_name);
    OpenDoor();
  }
  else
  {
    client.send("FACE NOT RECOGNISED");
    Serial.println("FACE NOT RECOGNISED");
    sendPhotoTelegram(0, "Unknown");
  }  
}

void HandleWebSocket(WebsocketsClient &client, http_img_process_result out_res, dl_matrix3du_t *image_matrix)
{
  fb = esp_camera_fb_get();

  if (g_state == START_DETECT || g_state == START_ENROLL || g_state == START_RECOGNITION)
  {
    out_res.net_boxes = NULL;
    out_res.face_id = NULL;

    fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image);

    out_res.net_boxes = face_detect(image_matrix, &mtmn_config);

    if (out_res.net_boxes)
    {
      if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK)
      {
        out_res.face_id = get_face_id(aligned_face);
        last_detected_millis = millis();
        if (g_state == START_DETECT) {
          client.send("FACE DETECTED");
        }

        if (g_state == START_ENROLL)
        {
          int left_sample_face = do_enrollment(&st_face_list, out_res.face_id);
          char enrolling_message[64];
          sprintf(enrolling_message, "SAMPLE NUMBER %d FOR %s", ENROLL_CONFIRM_TIMES - left_sample_face, st_name.enroll_name);
          client.send(enrolling_message);
          if (left_sample_face == 0)
          {
            ESP_LOGI(TAG, "Enrolled Face ID: %s", st_face_list.tail->id_name);
            g_state = START_STREAM;
            char captured_message[64];
            sprintf(captured_message, "FACE CAPTURED FOR %s", st_face_list.tail->id_name);
            client.send(captured_message);
            send_face_list(client);

          }
        }

        if (g_state == START_RECOGNITION  && (st_face_list.count > 0))
        {
          HandleCapture(client, out_res);
        }
        dl_matrix3d_free(out_res.face_id);
      }
    }
    else
    {
      if (g_state != START_DETECT) {
        client.send("NO FACE DETECTED");
      }
    }

    if (g_state == START_DETECT && millis() - last_detected_millis > 500) { // Detecting but no face detected
      client.send("DETECTING");
    }

  }

  client.sendBinary((const char *)fb->buf, fb->len);

  esp_camera_fb_return(fb);
  fb = NULL;  
}

void loop() {

  if(longPressActive)
  {
    app_httpserver_init();
    app_facenet_main();
    socket_server.listen(82);

    String welcome = "Camera Ready! Use 'http://";
    welcome += WiFi.localIP();
    welcome += "' to connect";
//    Serial.print("Camera Ready! Use 'http://");
//    Serial.print(WiFi.localIP());
//    Serial.println("' to connect");
    bot.sendMessage(CHAT_ID, welcome, "");
    
    auto client = socket_server.accept();
    client.onMessage(handle_message);
    image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);
    out_res = {0};
//    dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);
//    http_img_process_result out_res = {0};
    out_res.image = image_matrix->item;
    
    send_face_list(client);
    client.send("STREAMING");
    
    while (client.available()) {
      client.poll();
      HandleWebSocket(client, out_res, image_matrix);
    }
    socket_server.~WebsocketsServer();
    longPressActive = false;
    Serial.println("Disconnect");
  }
  else if(shortPressActive)
  {
    g_state = START_RECOGNITION;
    Serial.println("RECOGNISING");
    PreparePhoto = true;
    shortPressActive = false;
  }

  CekTombol();

  if(DoorOpen) {
    if(millis() - StartWait > TimeWaitDoor) {
      digitalWrite(SelenoidPin, LOW);
      DoorOpen = false;
      DoorSwitch = false;
    }
  }

  if(DoorSwitch) {
    if(digitalRead(DoorSwitchPin) == 0) {
      StartWait = millis();
    }
    else {
      digitalWrite(SelenoidPin, LOW);
      DoorOpen = false;
      DoorSwitch = false;
      socket_server.listen(82);
    }
  }
  
  if(PreparePhoto) {
    for(uint8_t x = 0; x < 3; x++)
    {
      digitalWrite(FlashPin, HIGH); delay(500);
      digitalWrite(FlashPin, LOW); delay(500);
    }
    PreparePhoto = false;
    sendPhoto = true;
  }
  
  if (sendPhoto) {
    Serial.println("Preparing photo");
    Capture(out_res);
    sendPhoto = false; 
  }

  if (millis() > lastTimeBotRan + botRequestDelay) {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages) {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }
}
