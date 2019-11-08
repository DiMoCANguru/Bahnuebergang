
/* ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <DiMo-CANguru@WEB.DE> wrote this file. As long as you retain this
 * notice you can do whatever you want with this stuff. If we meet some day,
 * and you think this stuff is worth it, you can buy me a beer in return
 * Gustav Wostrack
 * ----------------------------------------------------------------------------
 */

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include "CANguruDefs.h"
#include "EEPROM.h"
#include "esp32-hal-ledc.h"
#include "esp_system.h"
#include <esp_now.h>
#include <esp_wifi.h>
#include <Ticker.h>

#define CAN_FRAME_SIZE 13 /* Länge der CAN-Frames */

// config-Daten
// Parameter-Kanäle
enum Kanals
{
  Kanal00,
  Kanal01,
  Kanal02,
  Kanal03,
  Kanal04,
  Kanal05,
  Kanal06,
  Kanal07,
  endofKanals
};

Kanals CONFIGURATION_Status_Index = Kanal00;
// Zeigen an, ob eine entsprechende Anforderung eingegangen ist
bool CONFIG_Status_Request = false;
bool SYS_CMD_Request = false;
boolean statusPING;

// Timer
/* create a hardware timer */
hw_timer_t *timer = NULL;
const uint8_t timerNmbr = 0;

// Blinker
/* create a hardware timer */
hw_timer_t *blinker = NULL;
const uint8_t blinkerNmbr = 1;
bool notBlinking;

// Die Schranke kann die beiden Zustände haben
enum statusGate
{
  gate2Close,
  gate2Open
};

// Status Gate
statusGate gateStatus;

// Motoranschlüsse
const uint16_t motorUp = GPIO_NUM_4;
const uint16_t motorDn = GPIO_NUM_0;

// Speed
uint8_t speedDest;
uint8_t speedCurr;
uint8_t speedStart;
uint8_t speedHz;
uint8_t speedminHz;
uint8_t speedmaxHz;
const uint8_t speedmax = 100;
const uint8_t speedmin = 70;

// Blink LEDs
const uint16_t Blink_LED_PIN = GPIO_NUM_12;
const uint32_t openGateBlinkTime = 100000;
uint32_t openGateBlinkCnt;
uint8_t blinkHiLo;

// Relais
const uint16_t relaisTime = 40000;
uint16_t relaisCnt = 0;
bool relaisOn = false;

const uint16_t OLIMEX_REL1_PIN = GPIO_NUM_32;
const uint16_t OLIMEX_REL2_PIN = GPIO_NUM_33;
const uint16_t RELAY_GPIO = OLIMEX_REL1_PIN;

// events
uint16_t msec = 0;

#define VERS_HIGH 0x00 // Versionsnummer vor dem Punkt
#define VERS_LOW 0x01  // Versionsnummer nach dem Punkt

// Protokollkonstante
#define PROT MM_ACC

// EEPROM-Adressen
#define setup_done 0x47
// EEPROM-Belegung
const uint16_t adr_setup_done = 0x00;
const uint16_t adr_decoderadr = 0x01;
const uint16_t adr_speed = 0x02;
const uint16_t adr_status = 0x03;
const uint16_t adr_channel0 = 0x04;
const uint16_t adr_channel1 = 0x05;
const uint16_t adr_channel2 = 0x06;
const uint16_t adr_channel3 = 0x07;
const uint16_t lastAdr = adr_channel3 + 1;
const uint16_t EEPROM_SIZE = lastAdr;

// forward declaration
void switchGate(uint8_t contact);

#include "espnow.h"

// Funktion stellt sicher, dass keine unerlaubten Werte geladen werden können
uint8_t readValfromEEPROM(uint16_t adr, uint8_t val, uint8_t min, uint8_t max)
{
  uint8_t v = EEPROM.read(adr);
  if ((v >= min) && (v <= max))
    return v;
  else
    return val;
}

// der Blink-Timerr das Andreaskreuz
// mit notBlinking wird gesteuert, ob die LEDs blinken
void IRAM_ATTR onBlink()
{
  if (notBlinking == true)
    return;
  if (blinkHiLo == HIGH)
    blinkHiLo = LOW;
  else
    blinkHiLo = HIGH;
  digitalWrite(Blink_LED_PIN, blinkHiLo);
}

// mit diesem Timer werden die Halbwellen für den Synchronmotor erzeugt
// falls eine andere Frequenz als 50 Hz eingestellt ist, wird langsam auf
// die höhere Frequenz hingesteuert.
// wenn speedCurr == speedDest, dann ist die Zielfrequenz erreicht
// Weiterhin wird hier die Abfallzeit für das Relais erzeugt und je nach
// Status die LEDs für das Andreas eingeschaltet
//
void IRAM_ATTR onTimer()
{
  static uint8_t cnt = 0;
  static uint16_t cnt2 = 0;
  msec++;
  cnt2++;
  //  100 - 50
  if (cnt2 == 1000)
  {
    if (speedCurr > speedDest)
      speedCurr--;
    if (speedCurr < speedDest)
      speedCurr++;
    cnt2 = 0;
  }
  if (cnt == 0)
  {
    digitalWrite(motorUp, HIGH);
    digitalWrite(motorDn, LOW);
  }
  if (cnt == speedCurr)
  {
    digitalWrite(motorUp, LOW);
    digitalWrite(motorDn, HIGH);
  }
  cnt++;
  if (cnt >= 2 * speedCurr)
    cnt = 0;
  if (relaisOn == true)
  {
    relaisCnt++;
    if (relaisCnt >= relaisTime)
    {
      relaisOn = false;
      digitalWrite(RELAY_GPIO, LOW);
    }
  }
  if (gateStatus == gate2Close)
  {
    if (notBlinking == false)
      openGateBlinkCnt--;
    if (openGateBlinkCnt == 0)
    {
      notBlinking = true;
      blinkHiLo = LOW;
      digitalWrite(Blink_LED_PIN, blinkHiLo);
    }
  }
}

void setup()
{
  Serial.begin(bdrMonitor);
  Serial.println("\r\n\r\nCANguru - S c h r a n k e");
  // der Decoder strahlt mit seiner Kennung
  // damit kennt die CANguru-Bridge (der Master) seine Decoder findet
  startAPMode();
  // der Master wird registriert
  addMaster();
  WiFi.disconnect();
  // EEPROM wird initialisiert
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("failed to initialise EEPROM");
  }
  uint8_t setup_todo = EEPROM.read(adr_setup_done);
  if (setup_todo != setup_done)
  {
    // alles fürs erste Mal
    //
    // wurde das Setup bereits einmal durchgeführt?
    // dann wird dieser Anteil übersprungen
    // 47, weil das EEPROM (hoffentlich) nie ursprünglich diesen Inhalt hatte
    // setzt die decoderadresse anfangs auf 1
    params.decoderadr = minadr;
    EEPROM.write(adr_decoderadr, params.decoderadr);
    EEPROM.commit();
    // Geschwindigkeit
    speedDest = speedmax;
    EEPROM.write(adr_speed, speedDest);
    EEPROM.commit();
    // Zustand
    gateStatus = gate2Close;
    EEPROM.write(adr_status, gateStatus);
    EEPROM.commit();
    // Gleisbesetzmelderadressen
    for (uint8_t ch = 0; ch < maxCntChannels; ch++)
    {
      channels[ch] = lastRMAdr;
      EEPROM.write(adr_channel0 + ch, channels[ch]);
      EEPROM.commit();
    }
    // setup is done
    EEPROM.write(adr_setup_done, setup_done);
    EEPROM.commit();
  }
  else
  {
    // nach dem ersten Mal
    // Adresse
    params.decoderadr = readValfromEEPROM(adr_decoderadr, minadr, minadr, maxadr);
    // Geschwindigkeit
    speedDest = readValfromEEPROM(adr_speed, speedmax, speedmin, speedmax);
    // Zustand
    gateStatus = (statusGate)readValfromEEPROM(adr_status, gate2Close, gate2Close, gate2Open);
    // Gleisbesetzmelderadressen
    for (uint8_t ch = 0; ch < maxCntChannels; ch++)
    {
      channels[ch] = readValfromEEPROM(adr_channel0 + ch, lastRMAdr, firstRMAdr, lastRMAdr);
    }
  }
  // ab hier werden die Anweisungen bei jedem Start durchlaufen
  // Flags
  got1CANmsg = false;
  SYS_CMD_Request = false;
  statusPING = false;
  openGateBlinkCnt = 0;
  // sets the pins as outputs:
  pinMode(motorUp, OUTPUT);
  pinMode(motorDn, OUTPUT);
  pinMode(RELAY_GPIO, OUTPUT);
  digitalWrite(RELAY_GPIO, LOW);
  pinMode(Blink_LED_PIN, OUTPUT);
  digitalWrite(Blink_LED_PIN, LOW);
  relaisOn = false;
  if (gateStatus == gate2Open)
    notBlinking = false;
  else
    notBlinking = true;
  msec = 0;
  speedCurr = speedDest;
  /* Use 1st timer of 4 */
  /* 1 tick take 1/(80MHZ/80) = 1us so we set divider 80 and count up */
  timer = timerBegin(timerNmbr, 80, true);
  /* Attach onTimer function to our timer */
  timerAttachInterrupt(timer, &onTimer, true);
  /* Set alarm to call onTimer function every second 1 tick is 1us
  => 1 second is 1000000us */
  /* Repeat the alarm (third parameter) */
  timerAlarmWrite(timer, 100, true);
  /* Start an alarm */
  timerAlarmEnable(timer);
  blinker = timerBegin(blinkerNmbr, 80, true);
  timerAttachInterrupt(blinker, &onBlink, true);
  timerAlarmWrite(blinker, 750000, true);
  timerAlarmEnable(blinker);
  stillAliveBlinkSetup();
}

/*
  Response
  Bestimmt, ob CAN Meldung eine Anforderung oder Antwort oder einer
  vorhergehende Anforderung ist. Grundsätzlich wird eine Anforderung
  ohne ein gesetztes Response Bit angestoßen. Sobald ein Kommando
  ausgeführt wurde, wird es mit gesetztem Response Bit, sowie dem
  ursprünglichen Meldungsinhalt oder den angefragten Werten, bestätigt.
  Jeder Teilnehmer am Bus, welche die Meldung ausgeführt hat, bestätigt ein
  Kommando.
  Das Response Bit wird hier gesetzt, indem opFrame[1] um 1 erhöht wird.
  */
// schickt einen CAN-Frame an den Master
void sendCanFrame()
{
  // opFrame[4] ist die Anzahl der gesetzten Datenbytes
  // alle Bytes jenseits davon werden auf 0x00 gesetzt
  for (uint8_t i = CAN_FRAME_SIZE - 1; i < 8 - opFrame[4]; i--)
    opFrame[i] = 0x00;
  opFrame[1]++;
  // opFrame[2] und opFrame[3] enthalten den Hash, der zu Beginn des Programmes
  // einmal errechnet wird
  opFrame[2] = hasharr[0];
  opFrame[3] = hasharr[1];
  sendTheData();
}

/*
  CAN Grundformat
  Das CAN Protokoll schreibt vor, dass Meldungen mit einer 29 Bit Meldungskennung,
  4 Bit Meldungslänge sowie bis zu 8 Datenbyte bestehen.
  Die Meldungskennung wird aufgeteilt in die Unterbereiche Priorit�t (Prio),
  Kommando (Command), Response und Hash.
  Die Kommunikation basiert auf folgendem Datenformat:

  Meldungskennung
  Prio	2+2 Bit Message Prio			28 .. 25
  Command	8 Bit	Kommando Kennzeichnung	24 .. 17
  Resp.	1 Bit	CMD / Resp.				16
  Hash	16 Bit	Kollisionsaufl�sung		15 .. 0
  DLC
  DLC		4 Bit	Anz. Datenbytes
  Byte 0	D-Byte 0	8 Bit Daten
  Byte 1	D-Byte 1	8 Bit Daten
  Byte 2	D-Byte 2	8 Bit Daten
  Byte 3	D-Byte 3	8 Bit Daten
  Byte 4	D-Byte 4	8 Bit Daten
  Byte 5	D-Byte 5	8 Bit Daten
  Byte 6	D-Byte 6	8 Bit Daten
  Byte 7	D-Byte 7	8 Bit Daten
  */

// Mit testMinMax wird festgestellt, ob ein Wert innerhalb der
// Grenzen von min und max liegt
bool testMinMax(uint8_t oldval, uint8_t val, uint8_t min, uint8_t max)
{
  return (oldval != val) && (val >= min) && (val <= max);
}

// receiveKanalData dient der Parameterübertragung zwischen Decoder und CANguru-Server
// es erhält die evtuelle auf dem Server geänderten Werte zurück
void receiveKanalData()
{
  SYS_CMD_Request = false;
  uint8_t speedHz;
  uint8_t oldval;
  switch (opFrame[10])
  {
  // Kanalnummer #1 - Decoderadresse
  case 1:
  {
    oldval = params.decoderadr;
    params.decoderadr = opFrame[12];
    if (testMinMax(oldval, params.decoderadr, minadr, maxadr))
    {
      // speichert die neue Adresse
      EEPROM.write(adr_decoderadr, params.decoderadr);
      EEPROM.commit();
      // neue Adressen
    }
    else
    {
      params.decoderadr = oldval;
    }
  }
  break;
  // Kanalnummer #2 - speed
  case 2:
  {
    oldval = speedDest;
    speedHz = opFrame[12];
    speedDest = 1 / (0.0001 * speedHz * 2);
    if (testMinMax(oldval, speedDest, speedmin, speedmax))
    {
      // speichert die neue Adresse
      EEPROM.write(adr_speed, speedDest);
      EEPROM.commit();
    }
    else
    {
      speedDest = oldval;
    }
  }
  break;
  // Kanalnummer #3 - status
  case 3:
  {
    oldval = (statusGate)gateStatus;
    gateStatus = (statusGate)opFrame[12];
    if (testMinMax(oldval, gateStatus, gate2Close, gate2Open))
    {
      // speichert die neue Adresse
      EEPROM.write(adr_status, gateStatus);
      EEPROM.commit();
    }
    else
    {
      gateStatus = (statusGate)oldval;
    }
  }
  break;
  case 4:
  case 5:
  case 6:
  case 7:
  {
    oldval = channels[opFrame[10] - 4];
    channels[opFrame[10] - 4] = opFrame[12];
    if (testMinMax(oldval, channels[opFrame[10] - 4], firstRMAdr, lastRMAdr))
    {
      // speichert die neue Adresse
      switch (opFrame[10])
      {
      case 4:
        EEPROM.write(adr_channel0, channels[0]);
        break;
      case 5:
        EEPROM.write(adr_channel1, channels[1]);
        break;
      case 6:
        EEPROM.write(adr_channel2, channels[2]);
        break;
      case 7:
        EEPROM.write(adr_channel3, channels[3]);
        break;
      }
      EEPROM.commit();
    }
    else
    {
      channels[opFrame[10] - 4] = oldval;
    }
  }
  break;
  }
  //
  opFrame[11] = 0x01;
  opFrame[4] = 0x07;
  sendCanFrame();
}

// sendPING ist die Antwort der Decoder auf eine PING-Anfrage
void sendPING()
{
  statusPING = false;
  opFrame[1] = PING;
  opFrame[4] = 0x08;
  for (uint8_t i = 0; i < uid_num; i++)
  {
    opFrame[i + 5] = params.uid_device[i];
  }
  opFrame[9] = VERS_HIGH;
  opFrame[10] = VERS_LOW;
  opFrame[11] = DEVTYPE_GATE >> 8;
  opFrame[12] = DEVTYPE_GATE;
  sendCanFrame();
}

// in switchGate wird die Schranke auf Anforderung geschlossen oder geöffnet
void switchGate(uint8_t contact)
{
  enum tracks
  {
    oneTrack,
    twoTracks
  };
  static tracks howManyTracks = oneTrack;
  static uint8_t firstContact;
  static uint8_t prevContact = 99;
  static uint8_t howManyContacts = 0;
  // doppelmeldungen entfernen
  if (contact == prevContact)
    return;
  prevContact = contact;
  // ein- oder zweigleisig?
  if (howManyContacts < 2)
    switch (gateStatus)
    {
    case gate2Close:
      // ein neuer Zyklus beginnt
      // Schranke muss geschlossen werden
      firstContact = contact;
      break;
    case gate2Open:
      if ((((firstContact == 0) || (firstContact == 1)) &&
           ((contact == 0) || (contact == 1))) ||
          (((firstContact == 2) || (firstContact == 3)) &&
           ((contact == 2) || (contact == 3))))
      {
        // der 1. und 2. Kontakt kommen vom selben Gleis
        howManyTracks = oneTrack;
      }
      else
      {
        howManyTracks = twoTracks;
      }
      break;
    }
  howManyContacts++;
  if (((howManyTracks == twoTracks) && (howManyContacts == 2)) ||
      ((howManyTracks == twoTracks) && (howManyContacts == 3)))
  {
    return;
  }
  speedCurr = speedmax;
  relaisCnt = 0;
  relaisOn = true;
  digitalWrite(RELAY_GPIO, HIGH);
  notBlinking = false;
  blinkHiLo = LOW;
  if (gateStatus == gate2Open)
  {
    openGateBlinkCnt = openGateBlinkTime;
    gateStatus = gate2Close;
    // schließen und neuer Zyklus kann beginnen
    howManyContacts = 0;
    prevContact = 99;
  }
  else
  {
    gateStatus = gate2Open;
  }
  EEPROM.write(adr_status, gateStatus);
  EEPROM.commit();
}

// auf Anforderung des CANguru-Servers sendet der Decoder
// mit dieser Prozedur sendConfig seine Parameterwerte
void sendConfig()
{
  const uint8_t Kanalwidth = 8;
  const uint8_t numberofKanals = endofKanals - 1;

  const uint8_t NumLinesKanal00 = 5 * Kanalwidth;
  uint8_t arrKanal00[NumLinesKanal00] = {
      /*1*/ Kanal00, numberofKanals, (uint8_t)0, (uint8_t)0, (uint8_t)0, (uint8_t)0, (uint8_t)0, params.decoderadr,
      /*2.1*/ (uint8_t)highbyte2char(hex2dec(params.uid_device[0])), (uint8_t)lowbyte2char(hex2dec(params.uid_device[0])),
      /*2.2*/ (uint8_t)highbyte2char(hex2dec(params.uid_device[1])), (uint8_t)lowbyte2char(hex2dec(params.uid_device[1])),
      /*2.3*/ (uint8_t)highbyte2char(hex2dec(params.uid_device[2])), (uint8_t)lowbyte2char(hex2dec(params.uid_device[2])),
      /*2.4*/ (uint8_t)highbyte2char(hex2dec(params.uid_device[3])), (uint8_t)lowbyte2char(hex2dec(params.uid_device[3])),
      /*3*/ 'C', 'A', 'N', 'g', 'u', 'r', 'u', ' ',
      /*4*/ 'S', 'c', 'h', 'r', 'a', 'n', 'k', 'e',
      /*5*/ 0, 0, 0, 0, 0, 0, 0, 0};
  const uint8_t NumLinesKanal01 = 4 * Kanalwidth;
  uint8_t arrKanal01[NumLinesKanal01] = {
      // #2 - WORD immer Big Endian, wie Uhrzeit
      /*1*/ Kanal01, 2, 0, minadr, 0, maxadr, 0, params.decoderadr,
      /*2*/ 'M', 'o', 'd', 'u', 'l', 'a', 'd', 'r',
      /*3*/ 'e', 's', 's', 'e', 0, '1', 0, (maxadr / 100) + '0',
      /*4*/ (maxadr - (uint8_t)(maxadr / 100) * 100) / 10 + '0', (maxadr - (uint8_t)(maxadr / 10) * 10) + '0', 0, 'A', 'd', 'r', 0, 0};
  const uint8_t NumLinesKanal02 = 5 * Kanalwidth;
  uint8_t arrKanal02[NumLinesKanal02] = {
      // #2 - WORD immer Big Endian, wie Uhrzeit
      /*1*/ Kanal02, 2, 0, speedminHz, 0, speedmaxHz, 0, speedHz,
      /*2*/ 'G', 'e', 's', 'c', 'h', 'w', 'i', 'n',
      /*3*/ 'd', 'i', 'g', 'k', 'e', 'i', 't', 0,
      /*4*/ speedmin + '0', 0, (uint8_t)(speedmax / 10) + '0', speedmax - (speedmax / 10) * 10 + '0', 0, 'H', 'e', 'r',
      /*5*/ 't', 'z', 0, 0, 0, 0, 0, 0};
  const uint8_t NumLinesKanal03 = 5 * Kanalwidth;
  uint8_t arrKanal03[NumLinesKanal03] = {
      // #2 - WORD immer Big Endian, wie Uhrzeit
      /*1*/ Kanal03, 2, 0, gate2Close, 0, gate2Open, 0, gateStatus,
      /*2*/ 'o', 'f', 'f', 'e', 'n', ' ', '(', '0',
      /*3*/ ')', ' ', 'g', 'e', 's', 'c', 'h', 'l',
      /*4*/ ' ', '(', '1', ')', 0, '0', 0, '1',
      /*5*/ 0, '-', 0, 0, 0, 0, 0, 0};
  const uint8_t NumLinesKanal04 = 4 * Kanalwidth;
  uint8_t arrKanal04[NumLinesKanal04] = {
      // #2 - WORD immer Big Endian, wie Uhrzeit
      /*1*/ Kanal04, 2, 0, firstRMAdr, 0, lastRMAdr, 0, channels[0],
      /*2*/ 'K', 'o', 'n', 't', 'a', 'k', 't', ' ',
      /*3*/ '1', 0, '0', 0, '9', '9', 0, 'A',
      /*4*/ 'd', 'r', 0, 0, 0, 0, 0, 0};
  const uint8_t NumLinesKanal05 = 4 * Kanalwidth;
  uint8_t arrKanal05[NumLinesKanal05] = {
      // #2 - WORD immer Big Endian, wie Uhrzeit
      /*1*/ Kanal05, 2, 0, firstRMAdr, 0, lastRMAdr, 0, channels[1],
      /*2*/ 'K', 'o', 'n', 't', 'a', 'k', 't', ' ',
      /*3*/ '2', 0, '0', 0, '9', '9', 0, 'A',
      /*4*/ 'd', 'r', 0, 0, 0, 0, 0, 0};
  const uint8_t NumLinesKanal06 = 4 * Kanalwidth;
  uint8_t arrKanal06[NumLinesKanal06] = {
      // #2 - WORD immer Big Endian, wie Uhrzeit
      /*1*/ Kanal06, 2, 0, firstRMAdr, 0, lastRMAdr, 0, channels[2],
      /*2*/ 'K', 'o', 'n', 't', 'a', 'k', 't', ' ',
      /*3*/ '3', 0, '0', 0, '9', '9', 0, 'A',
      /*4*/ 'd', 'r', 0, 0, 0, 0, 0, 0};
  const uint8_t NumLinesKanal07 = 4 * Kanalwidth;
  uint8_t arrKanal07[NumLinesKanal07] = {
      // #2 - WORD immer Big Endian, wie Uhrzeit
      /*1*/ Kanal07, 2, 0, firstRMAdr, 0, lastRMAdr, 0, channels[3],
      /*2*/ 'K', 'o', 'n', 't', 'a', 'k', 't', ' ',
      /*3*/ '4', 0, '0', 0, '9', '9', 0, 'A',
      /*4*/ 'd', 'r', 0, 0, 0, 0, 0, 0};
  uint8_t NumKanalLines[numberofKanals + 1] = {
      NumLinesKanal00, NumLinesKanal01, NumLinesKanal02, NumLinesKanal03,
      NumLinesKanal04, NumLinesKanal05, NumLinesKanal06, NumLinesKanal07};

  uint8_t paket = 0;
  uint8_t outzeichen = 0;
  CONFIG_Status_Request = false;
  for (uint8_t inzeichen = 0; inzeichen < NumKanalLines[CONFIGURATION_Status_Index]; inzeichen++)
  {
    opFrame[1] = CONFIG_Status + 1;
    switch (CONFIGURATION_Status_Index)
    {
    case Kanal00:
    {
      opFrame[outzeichen + 5] = arrKanal00[inzeichen];
    }
    break;
    case Kanal01:
    {
      opFrame[outzeichen + 5] = arrKanal01[inzeichen];
    }
    break;
    case Kanal02:
    {
      opFrame[outzeichen + 5] = arrKanal02[inzeichen];
    }
    break;
    case Kanal03:
    {
      opFrame[outzeichen + 5] = arrKanal03[inzeichen];
    }
    break;
    case Kanal04:
    {
      opFrame[outzeichen + 5] = arrKanal04[inzeichen];
    }
    break;
    case Kanal05:
    {
      opFrame[outzeichen + 5] = arrKanal05[inzeichen];
    }
    break;
    case Kanal06:
    {
      opFrame[outzeichen + 5] = arrKanal06[inzeichen];
    }
    break;
    case Kanal07:
    {
      opFrame[outzeichen + 5] = arrKanal07[inzeichen];
    }
    break;
    case endofKanals:
    {
      // der Vollständigkeit geschuldet
    }
    break;
    }
    outzeichen++;
    if (outzeichen == 8)
    {
      opFrame[4] = 8;
      outzeichen = 0;
      paket++;
      opFrame[2] = 0x00;
      opFrame[3] = paket;
      sendTheData();
      delay(wait_time_small);
    }
  }
  //
  memset(opFrame, 0, sizeof(opFrame));
  opFrame[1] = CONFIG_Status + 1;
  opFrame[2] = hasharr[0];
  opFrame[3] = hasharr[1];
  opFrame[4] = 0x06;
  for (uint8_t i = 0; i < 4; i++)
  {
    opFrame[i + 5] = params.uid_device[i];
  }
  opFrame[9] = CONFIGURATION_Status_Index;
  opFrame[10] = paket;
  sendTheData();
  delay(wait_time_small);
}

// In dieser Schleife verbringt der Decoder die meiste Zeit
void loop()
{
  // die boolsche Variable got1CANmsg zeigt an, ob vom Master
  // eine Nachricht gekommen ist; der Zustand der Flags
  // entscheidet dann, welche Routine anschließend aufgerufen wird
  if (got1CANmsg)
  {
    got1CANmsg = false;
    if (statusPING)
    {
      sendPING();
    }
    if (SYS_CMD_Request)
    {
      receiveKanalData();
    }
    if (CONFIG_Status_Request)
    {
      speedHz = 1 / (0.0001 * speedDest * 2);
      speedmaxHz = 1 / (0.0001 * speedmin * 2);
      speedminHz = 1 / (0.0001 * speedmax * 2);
      sendConfig();
    }
  }
}
