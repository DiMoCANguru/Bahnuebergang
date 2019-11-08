
/* ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <DiMo-CANguru@WEB.DE> wrote this file. As long as you retain this
 * notice you can do whatever you want with this stuff. If we meet some day,
 * and you think this stuff is worth it, you can buy me a beer in return
 * Gustav Wostrack
 * ----------------------------------------------------------------------------
 */

#ifndef ESP_NOW_H
#define ESP_NOW_H

#include <Arduino.h>

const uint8_t WIFI_CHANNEL = 0;
const uint8_t macLen = 6;
const uint8_t LED_BUILTIN = GPIO_NUM_18;

// Ich nutze die  MAC-Adresse der spezifischen ESP32 Module,
// Für den SLAVE Mode muss der Decoder zunächst im WIFI_AP betrieben werden
// Die spezifische MAC-Adresse erhält man mit  WiFi.softAPmacAddress()

// die CntChannels sind die vier Gleisbesetztmelder, die die
// Schranke zum Öffnen oder Schließen bringen können
const uint8_t maxCntChannels = 4;
uint8_t channels[maxCntChannels];
const uint8_t firstRMAdr = 1;
const uint8_t lastRMAdr = 99;

enum statustype
{
  even = 0,
  odd
};
statustype currStatus;

// die masterCustomMac-Adresse wurde für alle willkürlich festgelegt
// somit kennt jeder die Adresse des Masters
const uint8_t masterCustomMac[] = {0x30, 0xAE, 0xA4, 0x89, 0x92, 0x71};

esp_now_peer_info_t master;
const esp_now_peer_info_t *masterNode = &master;
uint8_t opFrame[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
bool got1CANmsg = false;
byte cnt = 0;
String ssid0 = "CNgrSLV";
deviceparams params;
uint8_t hasharr[] = {0x00, 0x00};

Ticker tckr1;
const float tckr1Time = 0.025;

uint16_t secs;

//*********************************************************************************************************
// die beiden nächsten Routinen sind für das Blinken des Decoders zuständig
void timer1s()
{
  if (secs > 0)
  {
    if (secs % 2 == 0)
    {
      digitalWrite(LED_BUILTIN, LOW);
    }
    else
    {
      digitalWrite(LED_BUILTIN, HIGH);
    }
    secs--;
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void stillAliveBlinkSetup()
{
  tckr1.attach(tckr1Time, timer1s); // each sec
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
  secs = 0;
}

/*
Kollisionsfreiheit zum CS1 Protokoll:
Im CAN Protokoll der CS1 wird der Wert 6 für den "com-Bereich der ID",
dies sind die Bits 7..9, d.h. Highest Bit im Lowest-Byte (0b0xxxxxxx)
und die beiden Bits darüber (0bxxxxxx11), nicht benutzt. Diese
Bitkombination wird daher zur Unterscheidung fest im Hash verwendet.
Kollisionsauflösung:
Der Hash dient dazu, die CAN Meldungen mit hoher Wahrscheinlichkeit kollisionsfrei zu gestalten.
Dieser 16 Bit Wert wird gebildet aus der UID Hash.
Berechnung: 16 Bit High UID XOR 16 Bit Low der UID. Danach
werden die Bits entsprechend zur CS1 Unterscheidung gesetzt.
*/
void generateHash(uint8_t offset)
{
  uint32_t uid = UID_BASE + offset;
  params.uid_device[0] = (uint8_t)(uid >> 24);
  params.uid_device[1] = (uint8_t)(uid >> 16);
  params.uid_device[2] = (uint8_t)(uid >> 8);
  params.uid_device[3] = (uint8_t)uid;

  uint16_t highbyte = uid >> 16;
  uint16_t lowbyte = uid;
  uint16_t hash = highbyte ^ lowbyte;
  bitWrite(hash, 7, 0);
  bitWrite(hash, 8, 1);
  bitWrite(hash, 9, 1);
  hasharr[0] = hash >> 8;
  hasharr[1] = hash;
}

void printMac(uint8_t m[macLen])
{
  for (int ii = 0; ii < macLen - 1; ++ii)
  {
    Serial.print(m[ii], HEX);
    Serial.print(":");
  }
  Serial.print(m[macLen - 1], HEX);
}

// der Decoder strahlt mit dem Präfix und seiner Mac-Adresse
void startAPMode()
{
  Serial.println();
  Serial.println("WIFI Connect AP Mode");
  Serial.println("--------------------");
  WiFi.persistent(false); // Turn off persistent to fix flash crashing issue.
  WiFi.mode(WIFI_OFF);    // https://github.com/esp8266/Arduino/issues/3100
  WiFi.mode(WIFI_AP);

  // Connect to Wi-Fi
  String ssid1 = WiFi.softAPmacAddress();
  ssid0 = ssid0 + ssid1;
  char ssid[30];
  ssid0.toCharArray(ssid, 30);
  WiFi.softAP(ssid); // Name des Access Points
  Serial.println(ssid);
}

// Falls mal etwas schiefgeht
void printESPNowError(esp_err_t Result)
{
  if (Result == ESP_ERR_ESPNOW_NOT_INIT)
  {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  }
  else if (Result == ESP_ERR_ESPNOW_ARG)
  {
    Serial.println("Invalid Argument");
  }
  else if (Result == ESP_ERR_ESPNOW_INTERNAL)
  {
    Serial.println("Internal Error");
  }
  else if (Result == ESP_ERR_ESPNOW_NO_MEM)
  {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  }
  else if (Result == ESP_ERR_ESPNOW_NOT_FOUND)
  {
    Serial.println("Peer not found.");
  }
  else if (Result == ESP_ERR_ESPNOW_IF)
  {
    Serial.println("Interface Error.");
  }
  else
  {
    Serial.printf("\r\nNot sure what happened\t%d", Result);
  }
}

// Das ist die Empfangsroutine des Decoders; er erhält alle Nachrichten
// vom Master über ESPNow. Hier wird auch eine Erstauswertung
// der Nachricht vorgenommen und dem Hauptprogramm mit der Variablen
// got1CANmsg und einem Flag mitgeteilt, was weiterhein zu tun ist.
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  memcpy(opFrame, data, data_len);
  if (data_len == macLen + 1)
  {
    // nur beim handshaking; es werden die macaddress
    // und die devicenummer übermittelt und die macaddress zurückgeschickt
    esp_err_t sendResult = esp_now_send(master.peer_addr, opFrame, macLen);
    if (sendResult != ESP_OK)
    {
      printESPNowError(sendResult);
    }
    generateHash(opFrame[macLen]);
    return;
  }
  switch (opFrame[0x01])
  {
  case BlinkAlive:
    if (secs < 10)
      secs = 10;
    break;
  case PING:
  {
    statusPING = true;
    got1CANmsg = true;
    // alles Weitere wird in loop erledigt
  }
  break;
  // config
  case CONFIG_Status:
  {
    CONFIG_Status_Request = true;
    CONFIGURATION_Status_Index = (Kanals)opFrame[9];
    if (CONFIGURATION_Status_Index > 0 && secs < 100)
      secs = 100;
    got1CANmsg = true;
  }
  break;
  case SYS_CMD:
  {
    if (opFrame[9] == SYS_STAT)
    {
      SYS_CMD_Request = true;
      // alles Weitere wird in loop erledigt
      got1CANmsg = true;
    }
  }
  break;
  case SWITCH_ACC:
  {
    // Umsetzung nur bei gültiger Weichenadresse
    uint16_t _gate_address = (uint16_t)((opFrame[7] << 8) | opFrame[8]);
    // berechnet die _gate_address aus der Decoderadresse und der Protokollkonstante
    uint16_t _own_address = PROT + (params.decoderadr - 1) * 4;
    // Auf benutzte Adresse überprüfen
    if ((_own_address <= _gate_address) && ((_own_address + 4) >= _gate_address))
    {
      switchGate(_gate_address - _own_address);
    }
  }
  break;
  case S88_EVENT_R:
  {
    for (uint8_t ch = 0; ch < maxCntChannels; ch++)
    {
      if (channels[ch] == opFrame[0x08])
      {
        if (opFrame[0x0A] == 0x01)
          switchGate(ch);
        break;
      }
    }
  }
  break;
  }
}

// die Senderoutine
void sendTheData()
{
  esp_now_send(master.peer_addr, opFrame, CAN_FRAME_SIZE);
}

// Auswertung, ob die Sendung erfolgreich war
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "Fail");
}

// Hiermit wird der ESPNow initialisert, die Sende - und Empfangsroutine registiert
// sowieder Master angemeldet
void addMaster()
{
  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESPNow Init Success!");
  }
  else
  {
    Serial.println("ESPNow Init Failed....");
  }
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  memcpy(&master.peer_addr, &masterCustomMac, macLen);
  master.channel = WIFI_CHANNEL; // pick a channel
  master.encrypt = 0;            // no encryption
  //Add the master node to this slave node
  master.ifidx = ESP_IF_WIFI_AP;
  //Add the remote master node to this slave node
  if (esp_now_add_peer(masterNode) == ESP_OK)
  {
    Serial.println("Master Added As Peer!");
  }
}

#endif