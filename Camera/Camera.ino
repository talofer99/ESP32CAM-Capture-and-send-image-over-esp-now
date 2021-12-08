/*
// CAPTURE AND SEND IMAGE OVER ESP NOW
// Code by: Tal Ofer
// https://github.com/talofer99/ESP32CAM-Capture-and-send-image-over-esp-now
//
// This is the camera portion of the code.
//
// for more information
// https://www.youtube.com/watch?v=0s4Bm9Ar42U
*/


#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"

#include <esp_now.h>
#include <WiFi.h>
#define ONBOADLED 4
#define RXPIN 3
#include "esp_camera.h"

// Pin definition for CAMERA_MODEL_AI_THINKER
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

#define fileDatainMessage 240.0
#define UARTWAITHANDSHACK 1000
// Global copy of slave
esp_now_peer_info_t slave;
#define CHANNEL 1
#define PRINTSCANRESULTS 1
#define DELETEBEFOREPAIR 1

// for esp now connect
unsigned long lastConnectNowAttempt;
unsigned long nextConnectNowGap = 1000;
bool isPaired = 0;

// for photo name
int pictureNumber = 1;
byte takeNextPhotoFlag = 0;

// for photo transmit
int currentTransmitCurrentPosition = 0;
int currentTransmitTotalPackages = 0;
byte sendNextPackageFlag = 0;
String fileName = "/moon.jpg";

// for connection type
bool useUartRX = 0;


void setup() {
  // NEEDED ????
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  // start serial
  Serial.begin(115200);
  Serial.println("CAMERA MASTER TARTED");
  // init camera
  initCamera();
  // init sd
  initSD();

  // inid onboad led
  pinMode(ONBOADLED, OUTPUT);
  digitalWrite(ONBOADLED, LOW);


  // we now test to see if we got serial communication
  unsigned long testForUart = millis();
  Serial.print("WAIT UART");
  while (testForUart + UARTWAITHANDSHACK > millis() && !Serial.available())
  {
    Serial.print(".");
    delay(50);
  }
    
  if (Serial.available())
  {
    Serial.println("We are using Serial!!");
    while (Serial.available())
    {
      Serial.println(Serial.read());
    }
    useUartRX = 1;
  }


  if (!useUartRX)
  {
    // set RX as pullup for safety
    pinMode(RXPIN, INPUT_PULLUP);
    Serial.println("We are using the button");
    //Set device in STA mode to begin with
    WiFi.mode(WIFI_STA);
    // This is the mac address of the Master in Station Mode
    Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
    // Init ESPNow with a fallback logic
    InitESPNow();
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);
  }

}

void loop() {

  // if we are:
  // 1. NOT USING UART AS CONNECTION (ESP NOW WORKING)
  // 2. NOT PARIED
  // 3. OUR LAST CONNECT ATTMEPT WAS OVER DUE
  if (!useUartRX && !isPaired  && lastConnectNowAttempt + nextConnectNowGap < millis())
  {
    Serial.println("NOT CONNECTED -> TRY TO CONNECT");
    ScanAndConnectToSlave();
    // if we connected
    if (isPaired)
    {
      blinkIt(150, 2);
    }
    else
    {
      nextConnectNowGap *= 2; // dbl the gap
      blinkIt(150, 3); // blink 3 times
    }

    // save last attmpe
    lastConnectNowAttempt = millis();
  }

  // if we are :
  // 1. NOT using the UART - IE we control from the button
  // 2. The button is LOW (pressed)
  // 3. we are currently not have currentTransmitTotalPackages set
  // 4. the sendNextPackageFlag is not set.

  if (!useUartRX &&  !digitalRead(RXPIN) && !currentTransmitTotalPackages && !sendNextPackageFlag )
    takeNextPhotoFlag = 1;

  // if the sendNextPackageFlag is set
  if (sendNextPackageFlag)
    sendNextPackage();

  // if takeNextPhotoFlag is set
  if (takeNextPhotoFlag)
    takePhoto();


  // we only read serial if we use the uart
  if (Serial.available() && useUartRX)
  {
    switch (Serial.read())
    {
      case 'p':
      case 'P':
        takeNextPhotoFlag = 1;
        break;
      case 's':
      case 'S':
        ScanAndConnectToSlave();
        break;
      case 't':
      case 'T':
        startTransmit();
        break;
      default:
        Serial.println("not supported!!!");
        break;
    } //end switch
  } //end if
}



/* ***************************************************************** */
/*                  CAMERA RELATED FUNCTIONS                         */
/* ***************************************************************** */


/* ***************************************************************** */
/* TAKE PHOTO                                                        */
/* ***************************************************************** */
void takePhoto()
{
  takeNextPhotoFlag = 0;
  digitalWrite(4, HIGH);
  delay(50);
  camera_fb_t * fb = NULL;

  // Take Picture with Camera
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  digitalWrite(4, LOW);
  // Path where new picture will be saved in SD Card
  String path = "/picture" + String(pictureNumber) + ".jpg";

  fs::FS &fs = SD_MMC;
  Serial.printf("Picture file name: %s\n", path.c_str());

  fs.remove(path.c_str());

  File file = fs.open(path.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file in writing mode");
  }
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved file to path: %s\n", path.c_str());
  }
  file.close();
  esp_camera_fb_return(fb);

  fileName = path;
  if (isPaired)
    startTransmit();

  pictureNumber++;

}

/* ***************************************************************** */
/* INIT SD                                                           */
/* ***************************************************************** */
void initSD()
{
  Serial.println("Starting SD Card");
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD Card Mount Failed");
    return;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD Card attached");
    return;
  }
}



/* ***************************************************************** */
/* INIT CAMERA                                                       */
/* ***************************************************************** */
void initCamera()
{
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

  Serial.println("psramFound() = " + String(psramFound()));

  if (psramFound()) {
    config.frame_size = FRAMESIZE_QVGA; //FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA //FRAMESIZE_QVGA
    config.jpeg_quality = 2;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}





/* ***************************************************************** */
/*                  ESP NOW RELATED FUNCTIONS                        */
/* ***************************************************************** */


/* ***************************************************************** */
/* START TRASMIT                                                     */
/* ***************************************************************** */
void startTransmit()
{
  Serial.println("Starting transmit");
  fs::FS &fs = SD_MMC;
  File file = fs.open(fileName.c_str(), FILE_READ);
  if (!file) {
    Serial.println("Failed to open file in writing mode");
    return;
  }
  Serial.println(file.size());
  int fileSize = file.size();
  file.close();
  currentTransmitCurrentPosition = 0;
  currentTransmitTotalPackages = ceil(fileSize / fileDatainMessage);
  Serial.println(currentTransmitTotalPackages);
  uint8_t message[] = {0x01, currentTransmitTotalPackages >> 8, (byte) currentTransmitTotalPackages};
  sendData(message, sizeof(message));
}

/* ***************************************************************** */
/* SEND NEXT PACKAGE                                                 */
/* ***************************************************************** */
void sendNextPackage()
{
  // claer the flag
  sendNextPackageFlag = 0;

  // if got to AFTER the last package
  if (currentTransmitCurrentPosition == currentTransmitTotalPackages)
  {
    currentTransmitCurrentPosition = 0;
    currentTransmitTotalPackages = 0;
    Serial.println("Done submiting files");
    //takeNextPhotoFlag = 1;
    return;
  } //end if

  //first read the data.
  fs::FS &fs = SD_MMC;
  File file = fs.open(fileName.c_str(), FILE_READ);
  if (!file) {
    Serial.println("Failed to open file in writing mode");
    return;
  }

  // set array size.
  int fileDataSize = fileDatainMessage;
  // if its the last package - we adjust the size !!!
  if (currentTransmitCurrentPosition == currentTransmitTotalPackages - 1)
  {
    Serial.println("*************************");
    Serial.println(file.size());
    Serial.println(currentTransmitTotalPackages - 1);
    Serial.println((currentTransmitTotalPackages - 1)*fileDatainMessage);
    fileDataSize = file.size() - ((currentTransmitTotalPackages - 1) * fileDatainMessage);
  }

  //Serial.println("fileDataSize=" + String(fileDataSize));

  // define message array
  uint8_t messageArray[fileDataSize + 3];
  messageArray[0] = 0x02;


  file.seek(currentTransmitCurrentPosition * fileDatainMessage);
  currentTransmitCurrentPosition++; // set to current (after seek!!!)
  //Serial.println("PACKAGE - " + String(currentTransmitCurrentPosition));

  messageArray[1] = currentTransmitCurrentPosition >> 8;
  messageArray[2] = (byte) currentTransmitCurrentPosition;
  for (int i = 0; i < fileDataSize; i++)
  {
    if (file.available())
    {
      messageArray[3 + i] = file.read();
    } //end if available
    else
    {
      Serial.println("END !!!");
      break;
    }
  } //end for

  sendData(messageArray, sizeof(messageArray));
  file.close();

}


/* ***************************************************************** */
/* SEND DATA                                                         */
/* ***************************************************************** */
void sendData(uint8_t * dataArray, uint8_t dataArrayLength) {
  const uint8_t *peer_addr = slave.peer_addr;
  //Serial.print("Sending: "); Serial.println(data);
  //Serial.print("length: "); Serial.println(dataArrayLength);

  esp_err_t result = esp_now_send(peer_addr, dataArray, dataArrayLength);
  //Serial.print("Send Status: ");
  if (result == ESP_OK) {
    //Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

/* ***************************************************************** */
/* callback when data is sent from Master to Slave                   */
/* ***************************************************************** */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

  if (currentTransmitTotalPackages)
  {
    sendNextPackageFlag = 1;
    // if nto suecess 0 resent the last one
    if (status != ESP_NOW_SEND_SUCCESS)
      currentTransmitCurrentPosition--;
  } //end if
}






/* ***************************************************************** */
/* Init ESP Now with fallback                                        */
/* ***************************************************************** */
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}



/* ***************************************************************** */
/* Scan for slaves in AP mode                                        */
/* ***************************************************************** */
void ScanAndConnectToSlave() {
  int8_t scanResults = WiFi.scanNetworks();
  // reset on each scan
  bool slaveFound = 0;
  memset(&slave, 0, sizeof(slave));

  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        Serial.println("Found a Slave.");
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x%c",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slave.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }

        slave.channel = CHANNEL; // pick a channel
        slave.encrypt = 0; // no encryption

        slaveFound = 1;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (slaveFound) {
    Serial.println("Slave Found, processing..");
    if (slave.channel == CHANNEL) { // check if slave channel is defined
      // `slave` is defined
      // Add slave as peer if it has not been added already
      isPaired = manageSlave();
      if (isPaired) {
        Serial.println("Slave pair success!");
      } else {
        // slave pair failed
        Serial.println("Slave pair failed!");
      }
    }
  } else {
    Serial.println("Slave Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
} //end functin

/* ***************************************************************** */
/* Check if the slave is already paired with the master.             */
/* If not, pair the slave with master                                */
/* ***************************************************************** */
bool manageSlave() {
  if (slave.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

    Serial.print("Slave Status: ");
    const esp_now_peer_info_t *peer = &slave;
    const uint8_t *peer_addr = slave.peer_addr;
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(peer_addr);
    if ( exists) {
      // Slave already paired.
      Serial.println("Already Paired");
      return true;
    } else {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(peer);
      if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Pair success");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        Serial.println("Peer list full");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("Out of memory");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("Peer Exists");
        return true;
      } else {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
    return false;
  }
}


/* ***************************************************************** */
/* DELETE PEER                                                       */
/* ***************************************************************** */
void deletePeer() {
  const esp_now_peer_info_t *peer = &slave;
  const uint8_t *peer_addr = slave.peer_addr;
  esp_err_t delStatus = esp_now_del_peer(peer_addr);
  Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}



/* ***************************************************************** */
/*                  HELPERS RELATED FUNCTIONS                        */
/* ***************************************************************** */



void blinkIt(int delayTime, int times)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite(ONBOADLED, HIGH);
    delay(delayTime);
    digitalWrite(ONBOADLED, LOW);
    delay(delayTime);
  }
}
