/// @dir pca301 (2014-07-30)
/// a PCA 301 communicator
//
// authors: ohweh + justme1968 + trilu
// see http://forum.fhem.de/index.php?t=msg&goto=91926&rid=0
//
// This code is derived from RF12demo.10 being part of "JeeLib"
// see https://github.com/jcw/jeelib
// 2009-05-06 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
//
// History of changes:
// 2013-08-01 - started working on PCA 301 decoding with Trilu and Kleiner
// 2013-08-02 - output leading spaces while using hex report format
// 2013-08-03 - added crc check for PCA 301 packets
// 2013-08-03 - on valid crc print "OK 24" to packets from node 0-10 in group 212
// 2013-08-04 - due to needed modification, JeeLib RF12 code has been embedded
// 2013-08-05 - all RF12 functions offloaded into new library pca301
// 2013-08-05 - menu cleanup
// 2013-08-05 - implemented sending
// 2013-08-07 - added pairing functionality
// 2013-08-08 - turn on/off added to menu
// 2013-08-10 - implemented fill/load/save/erase config
// 2013-08-10 - added polling scheduler
// 2013-08-14 - in quiet mode, suppress packets from other polling units (except "switch")
// 2013-08-17 - add crc check for eeprom config load/save
// 2013-08-17 - added static node id 24 (hex 18) as descriptor for PCA301 (needed by FHEM)
// 2013-08-21 - in TX packets: changed tailing 4x170 towards 4x255
// 2013-08-25 - added PROGNAME + PROGVERS to showHelp()
// 2013-09-02 - trigger poll after switch command
// 2014-05-05 - set command for center frequency implemented in: static void handleInput (char c)
// 2014-07-30 - changed version to v10.1, changed name to pca10serial.ino, corrected several typos (PeMue)
// 2017-01-27 - <dev@mcbachmann.de> adapted pca301serial code to RFM69
//

//
// Description and packet structure (as described by "Kleiner"):
//
// - rf module is a RFM12BP from Hope
// - handheld display unit polls device every 60s, device replies back
// - handheld display unit automatically resets device at 0am
//
// TX protocol (handheld display unit):
// TX: AA AA AA 2D D4 01 04 07 F8 92 00 AA AA AA AA 71 52 AA AA AA
// TX: AA AA AA 2D D4 01 04 07 F8 92 00 AA AA AA AA 71 52 AA AA AA
//
// TX: AA AA AA 2D D4 01 05 07 F8 92 01 AA AA AA AA 77 4A AA AA AA
//
// RX protocol (handheld display unit):
// RX: 01 04 07 F8 92 00 00 00 00 00 0E 9F
// RX: 01 04 07 F8 92 01 00 00 00 00 8E E4
//
// RX: 01 05 07 F8 92 01 AA AA AA AA 77 4A
//
// Interpretation:
// 1 Byte: channel
// 1 Byte: command (04=retrieve measure data, 05=switch device, 06=identify device by toggling device LED
// 3 Byte: device address (UID) 
// 1 Byte: data    -> 1 with command=4 resets device statistics
//                 -> 0/1 with command=5 switches device off/on
// 2 Byte: current consumption in watt (scale 1/10)
// 2 Byte: total consumption in kWh (scale 1/100)
// 2 Byte: CRC16 (CRC16 XMODEM with Polynom 8005h)
//

#include <SPI.h>
#include "funky_rfm69.h"
#include "pca301_rfm69.h"

#define SERIAL_BAUD      57600
#define LED_PIN          9               // activity LED, comment out to disable

#define PROGNAME         "pcaSerial"
#define PROGVERS         "10.1"
#define NODEID           24

#define RF_MAX   (RFM69_MAXDATA + 5)    // maximum transmit / receive buffer: 3 header + data + 2 crc bytes
#define RF_FREQ_BASE     868000         // frequency base


//- variables --------------------------------------------------------------------------------------
static char cmd;
static String freq;
static byte value, stack[RFM69_MAXDATA+4], top, sendLen;
static byte pBuf[RFM69_MAXDATA], mode;
struct_pcaConf pcaConf;
uint16_t eeprom_crc;                     // eeprom crc
uint16_t rfm69_crc = 0;                  // running crc value
uint8_t  rfm69_buf[RF_MAX];              // recv/xmit buf, including hdr & crc bytes
uint8_t  rxfill = 0;                     // RX fill level
uint8_t  rfm69_len = 7;                  // fixed calculation value
uint32_t rfm69_center_freq = 868950;     // center frequency


//- prototypes -------------------------------------------------------------------------------------
static void sendDevice(uint8_t devPtr, char cmd);
static void showByte (byte value);
static uint8_t getDevice(uint32_t devId);
static uint32_t mem2devId(volatile uint8_t * data);
static uint32_t mem2long(volatile uint8_t * data);
static uint16_t mem2word(volatile uint8_t * data);
static void displayVersion(uint8_t newline);
static uint16_t hexToUInt16(String hexString);
static byte loadConf();
static void saveConf();
static void eraseConf();
static void fillConf();
static uint16_t crc16_pca301_update(uint16_t crc, uint8_t data);


//- report pcaConf ---------------------------------------------------------------------------------
void reportConf(uint8_t repMode) {
  for (int i = 0; i < pcaConf.numDev; i++) {
    switch (repMode) {
      case 1:
        Serial.print("L ");
        Serial.print(NODEID);
        Serial.print(' ');
        Serial.print(i+1);
        Serial.print(' ');
        break;
      case 2:
        Serial.print("R ");
        break;
      default:
        break;
    }
    Serial.print(pcaConf.pcaDev[i].retries);
    Serial.print(" : ");
    Serial.print(pcaConf.pcaDev[i].channel);
    Serial.print(" 4 ");
    Serial.print((byte)(pcaConf.pcaDev[i].devId >> 16));
    Serial.print(' ');
    Serial.print((byte)(pcaConf.pcaDev[i].devId >> 8));
    Serial.print(' ');
    Serial.print((byte)(pcaConf.pcaDev[i].devId));
    Serial.print(' ');
    Serial.print(pcaConf.pcaDev[i].pState);
    Serial.print(' ');
    Serial.print((byte)(pcaConf.pcaDev[i].pNow >> 8));
    Serial.print(' ');
    Serial.print((byte)(pcaConf.pcaDev[i].pNow));
    Serial.print(' ');
    Serial.print((byte)(pcaConf.pcaDev[i].pTtl >> 8));
    Serial.print(' ');
    Serial.print((byte)(pcaConf.pcaDev[i].pTtl));
    Serial.println();
  }
}

//- modify pcaConf ---------------------------------------------------------------------------------
void modifyConf(volatile uint8_t value) {
  switch (value) {
    case 0: fillConf();  break;
    case 1: loadConf();  break;
    case 2: saveConf();  break;
    case 3: eraseConf(); break;
  };
}

//- pcaTask ----------------------------------------------------------------------------------------
void pcaTask() {
  for (int i = 0; i < pcaConf.numDev; i++) {
    if (millis() / 100 > pcaConf.pcaDev[i].nextTX) {
      if (pcaConf.pcaDev[i].retries <= 255)
        pcaConf.pcaDev[i].retries += 1;
      if (pcaConf.pcaDev[i].retries < PCA_MAXRETRIES)
        pcaConf.pcaDev[i].nextTX = millis() / 100 + random(0,30) + 10;
      else
        pcaConf.pcaDev[i].nextTX = millis() / 100 + random(0,30) + pcaConf.deadIntv;
      sendDevice(i+1,'p');
      cmd = 'p';
      return;
    }
  }
}
  
//- send device ------------------------------------------------------------------------------------
void sendDevice(uint8_t devPtr, char cmd) {
  
  if (--devPtr >= 0 && devPtr < pcaConf.numDev) {
    pBuf[0] = pcaConf.pcaDev[devPtr].channel;
    switch (cmd) {
      case    'p': pBuf[1] = 4;  break;   // poll
      case    'j': pBuf[1] = 17; break;   // pair
      default    : pBuf[1] = 5;           // switch
    }
    pBuf[2] = pcaConf.pcaDev[devPtr].devId >> 16;
    pBuf[3] = pcaConf.pcaDev[devPtr].devId >> 8;
    pBuf[4] = pcaConf.pcaDev[devPtr].devId;

    if (cmd == 'e')
      pBuf[5] = 1;  // turn "on" (with byte 1 set to 5)
    else
      pBuf[5] = 0;  // turn "off" (with byte 1 set to 5)
    pBuf[6] = pBuf[7] = pBuf[8] = pBuf[9] = 0xFF;

    sendLen = 10;

    if (!pcaConf.quiet) {
      Serial.print("TX ");
      Serial.print(NODEID);
      for (byte i = 0; i < sendLen; i++) {
        Serial.print(' ');
        showByte(pBuf[i]);
      }
      Serial.println();
    }
    
  }
}

//- set next tx time for a given device ------------------------------------------------------------
void setNextTX (uint32_t devId, uint8_t nextTX) {
  uint8_t devPtr = getDevice(devId);
  if (devPtr)
    pcaConf.pcaDev[devPtr-1].nextTX  = millis() / 100 + nextTX;
  return;
}

//- analyze packet ---------------------------------------------------------------------------------
static void analyzePacket (void) {

  uint32_t devId = mem2devId(rfm69_buf+2);
  uint8_t devPtr = getDevice(devId);
  byte confChanged = 0;

  //- unknown device? add it to pcaConf ------------------------------------------------------------
  if (!devPtr) {
    devPtr = ++pcaConf.numDev;
    pcaConf.pcaDev[devPtr-1].devId = devId;
    //- is this device already paired with a handheld display unit? --------------------------------
    if (rfm69_buf[0]) {
      //- device is paired to handheld display unit, therefore use same channel --------------------
      pcaConf.pcaDev[devPtr-1].channel = rfm69_buf[0];
    } else {
      //- device is not paired to an handheld display unit, assign a free channel ------------------
      pcaConf.pcaDev[devPtr-1].channel = pcaConf.numDev;
    }
    confChanged = 1;
  } else if (rfm69_buf[0] && pcaConf.pcaDev[devPtr-1].channel != rfm69_data[0]) {
      //- known device, but used channel is different -> update config in memory -------------------
      pcaConf.pcaDev[devPtr-1].channel = rfm69_buf[0];
      confChanged = 1;
  }

  //- update dynamic values ------------------------------------------------------------------------
  if (mem2long(rfm69_buf+6) != 0xAAAAAAAA && mem2long(rfm69_data+6) != 0xFFFFFFFF) {
    pcaConf.pcaDev[devPtr-1].pState  = rfm69_buf[5]; 
    pcaConf.pcaDev[devPtr-1].pNow    = mem2word(rfm69_buf+6);
    pcaConf.pcaDev[devPtr-1].pTtl    = mem2word(rfm69_buf+8);
    pcaConf.pcaDev[devPtr-1].nextTX  = millis() / 100 + random(0,30) + pcaConf.pollIntv;
    pcaConf.pcaDev[devPtr-1].retries = 0;
  } else if (rfm69_buf[1] == 5) {
    // switch command, trigger poll
    pcaConf.pcaDev[devPtr-1].nextTX  = millis() / 100 + 5;
  }

  //- pairing request received? --------------------------------------------------------------------
  if (!rfm69_buf[0]) {
    if (!pcaConf.quiet) {
      Serial.print("#PREQ ");
      Serial.println(devId);
    }
    delay(70);                   // there's a timing issue while pairing, lose a bit of time
    sendDevice(devPtr,'j');
    cmd = 'j';
  }

  //- save config to EEPROM ------------------------------------------------------------------------
  if (confChanged)
    saveConf();

} // analyzePacket

//- lookup device ----------------------------------------------------------------------------------
static uint8_t getDevice(uint32_t devId) {
  for (int i = 0; i < pcaConf.numDev; i++) {
    if (pcaConf.pcaDev[i].devId == devId)
      return(i+1);    // device found
  }
  return 0;
}

//- get devId --------------------------------------------------------------------------------------
static uint32_t mem2devId(volatile uint8_t * data) {
  return (uint32_t)data[0] << 16 | (uint32_t)data[1] << 8 | (uint32_t)data[2];
}

//- mem2word ---------------------------------------------------------------------------------------
static uint16_t mem2word(volatile uint8_t * data) {
  return data[0] << 8 | data[1];
}

//- mem2long ---------------------------------------------------------------------------------------
static uint32_t mem2long(volatile uint8_t * data) {
  return (uint32_t)mem2word(data+0) << 16 | mem2word(data+2);
}

//- turn activityLed on/off ------------------------------------------------------------------------
static void activityLed (byte on) {
  #ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, !on);
  #endif
}

//- showNibble -------------------------------------------------------------------------------------
static void showNibble (byte nibble) {
  char c = '0' + (nibble & 0x0F);
  if (c > '9')
    c += 7;
  Serial.print(c);
}

//- showByte ---------------------------------------------------------------------------------------
static void showByte (byte value) {
  Serial.print(value);
}

//- helpText ---------------------------------------------------------------------------------------
const char helpText1[] PROGMEM = 
  "\n"
  "Available commands:" "\n"
  "     ..,.. s    - send data packet" "\n"
  "           l    - list devices" "\n"
  "       <n> a    - turn activity LED on PB1 on or off" "\n"
  "       <n> c    - config (0=fill, 1=load, 2=save, 3=erase)" "\n"
  "       <n> d    - turn off device <n>" "\n"
  "       <n> e    - turn on device <n>" "\n"
  "  0x<hhhh> h    - set center frequency offset (Example: 0x03B6 => 868.950MHz)" "\n"
  "                  note: leading zeros must be entered" "\n"
  "       <n> p    - poll device <n>" "\n"
  "       <n> r    - list recordings" "\n"
  "       <n> q    - quiet mode (1=suppress TX and bad packets)" "\n"
  "       <n> v    - version and configuration report" "\n"
;

//- showString -------------------------------------------------------------------------------------
static void showString (PGM_P s) {
  for (;;) {
    char c = pgm_read_byte(s++);
    if (c == 0)
      break;
    if (c == '\n')
      Serial.print('\r');
    Serial.print(c);
  }
}

//- showHelp ---------------------------------------------------------------------------------------
static void showHelp () {
  Serial.print("\n[");
  Serial.print(PROGNAME);
  Serial.print('.');
  Serial.print(PROGVERS);
  Serial.println(']');
  showString(helpText1);
  Serial.println();
}

//- handleInput ------------------------------------------------------------------------------------
static void handleInput (char c) {
  if (freq.startsWith("0x") && c != 'h') {
    if (('A' <= c && c <='F') || ('0' <= c && c <= '9')) {
      freq += c;
    }
  } else if ('0' <= c && c <= '9') {
    value = 10 * value + c - '0';
  } else if (c == ',') {
    if (top < sizeof stack)
      stack[top++] = value;
    value = 0;
  } else if (c == 'x') {
    freq = String("0x");
    value = 0;
  } else if ('a' <= c && c <='w') {      
      switch (c) {
        default:
          showHelp();
          break;
        case 'a':     // turn activity LED on or off
          activityLed(value);
          break;
        case 'l':     // list known devices
          reportConf(1);
          break;
        case 'q':     // turn quiet mode on or off (don't report TX and bad packets)
          pcaConf.quiet = value;
          break;
        case 'r':     // list recordings
          reportConf(2);
          break;
        case 's':     // send packet
          if (top < sizeof stack) {
            stack[top++] = value;
            sendLen      = top;
            cmd          = c;
            memcpy(pBuf, stack, top);
            if (sendLen == 10 && pBuf[1] == 5)
              setNextTX(mem2devId(pBuf+2), 10);
            if (!pcaConf.quiet) {
              Serial.print("TX ");
              Serial.print(NODEID);
              for (byte i = 0; i < sendLen; i++) {
                Serial.print(' ');
                showByte(pBuf[i]);
              }
              Serial.println();
            }
          } else
            top = 0;
          break;
        case 'v':     // report version and configuration parameters
          displayVersion(1);
          break;
        case 'd':     // turn a device off (disable)
        case 'e':     // turn a device on (enable)
        case 'p':     // poll a device
          sendDevice(value,c);
          cmd = c;
          break;
        case 'c':     // config options
          modifyConf(value);
          break;
        case 'h': // modify and display RFM69 Frequency register
          Serial.print("> FREQ set to: ");
          rfm69_center_freq = RF_FREQ_BASE + hexToUInt16(freq);
          rfm69_freq_carrier_khz(rfm69_center_freq);
          Serial.println(rfm69_center_freq);
          freq = String("");
          break;
      }
      value = top = 0;
      memset(stack, 0, sizeof stack);
  } else if (c == '+' || c == '-' || c == '#') {
    switch (c) {
      case '+': // modify and display RFM69 Frequency register
      case '-': // modify and display RFM69 Frequency register
        Serial.print("> FREQ");
        if (c == '+')
          rfm69_center_freq += 1;
        else
          rfm69_center_freq -= 1;
        rfm69_freq_carrier_khz(rfm69_center_freq);
        Serial.print(c);
        Serial.print(": "); 
        Serial.println(rfm69_center_freq);
        break;
      case '#': // test
        break;
    }
    value = 0;
    freq = String("");
  } else if (' ' < c && c < 'A')
    showHelp();
}

uint16_t hexToUInt16(String hexString) {
  
  uint16_t UInt16Value = 0;
  int nextInt;
  
  for (int i = 0; i < hexString.length(); i++) {
    
    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);
    
    UInt16Value = (UInt16Value * 16) + nextInt;
  }
  
  return UInt16Value;
}

void displayVersion(uint8_t newline) {
  Serial.print("\n[");
  Serial.print(PROGNAME);
  Serial.print('.');
  Serial.print(PROGVERS);
  Serial.print(']');  
  if (newline!=0)
    Serial.println();
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// M A I N
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//- setup ------------------------------------------------------------------------------------------
void pca301serial_setup() {
    
  // switch off LED
  activityLed(0);

  // available cli options
  showHelp();

  // try loading config from EEPROM. if CRC does not match, use blank default config
  if (!loadConf())
    fillConf();

  // quiet is default
  pcaConf.quiet  = 1;

}


//- loop -------------------------------------------------------------------------------------------
void pca301serial_loop_pre() {
  uint16_t crc;

  if (rfm69_rx_avail()) {

    rfm69_crc = 0;
    while (true == rfm69_rx_avail()) {
      uint8_t in = rfm69_fifo_data();

      rfm69_buf[rxfill++] = in;
      if (rxfill <= 10) {
          rfm69_crc = crc16_pca301_update(rfm69_crc, in);
      }
    }

    /* compare CRC */
    if (rxfill) {
      crc = (rfm69_buf[10] << 8) | rfm69_buf[11];
      if (crc == rfm69_crc) {
          rfm69_crc = 0;
      }
    }
  }
}


//- loop -------------------------------------------------------------------------------------------
void pca301serial_loop() {
  uint8_t cnt;

  pca301serial_loop_pre();

  if (Serial.available()) {
    handleInput(Serial.read());
  }

  if (!cmd) {
//    pcaTask();                 // check tasks
  }

  if ((RFM69_OPMODE_RX == rfm69_opmode_get()) && rxfill) {

    /* clear fifo */
    rfm69_fifo_clear();

    if (rfm69_len > RFM69_MAXDATA) {
      rfm69_crc = 1;   // force bad crc if packet length is invalid
      Serial.println("bad CRC");

    }

    byte n = 10;               // fixed packet length
    if (rfm69_crc == 0) {

      // in quiet mode, suppress as much packets as possible from non-PCA301 transmitters
      if (pcaConf.quiet && rfm69_buf[0] != 0) {
        // quiet mode and not a pairing request
        if (mem2long(rfm69_buf+6) == 0xFFFFFFFF) {
          // originator is another JeeLink
          rxfill = 0;
          rfm69_crc = 0;
          return;
        }
        if (rfm69_buf[1] != 5 && mem2long(rfm69_data+6) == 0xAAAAAAAA) {
          // originator is a hardware display unit
          rxfill = 0;
          rfm69_crc = 0;
          return;
        }
        // all non PCA301 packets filtered EXCEPT switch command from hardware display unit      
      }
      activityLed(1);
      Serial.print("OK");
    } else {
      if (pcaConf.quiet) {     // don't report bad packets in quiet mode
        rxfill = 0;
        rfm69_crc = 0;
        return;
      }
      Serial.print(" ?");
    }

    Serial.print(' ');
    Serial.print(NODEID);

    // FHEM quick fix - unpaired devices get listed with channel 0
      Serial.print(' ');
    if (rfm69_buf[0] == 0) {
      showByte(pBuf[1]);
    } else {
      showByte(rfm69_buf[0]);
    }

    for (byte i = 1; i < n; ++i) {
      Serial.print(' ');
      showByte(rfm69_buf[i]);
    }

    Serial.println();
    activityLed(0);

    if (rfm69_crc == 0)
      analyzePacket();

    rxfill = 0;
    rfm69_crc = 0;
  }

  if (cmd) {
    activityLed(1);

    /* calculate CRC */
    rfm69_crc = 0;
    for (cnt = 0; cnt < sendLen; cnt++) {
        rfm69_crc = crc16_pca301_update(rfm69_crc, pBuf[cnt]);
    }

    /* add CRC to data stream */
    pBuf[sendLen++] = rfm69_crc >> 8;
    pBuf[sendLen++] = rfm69_crc & 0xff;

    rfm69_send(sendLen, pBuf);
    cmd = 0;
    sendLen = 0;
    activityLed(0);
  }
}


//- load config from EEPROM - returns 1 if valid config was found, otherwise 0
static byte loadConf() {
  uint16_t len   = sizeof(pcaConf);
  byte *pPtrByte = (byte*)&pcaConf;        // byte Ptr to pcaConf
  eeprom_crc     = 0;
  eeprom_read_block(&pcaConf, (void *) 0, len);

  for (int i=0; i < (len - 2); i++) {
    eeprom_crc = crc16_pca301_update(eeprom_crc, *pPtrByte);
    pPtrByte++;
  }

  // valid config in EEPROM?
  if (eeprom_crc == pcaConf.crc) {
    // valid config found, reset dynamic settings
    for (int i = 0; i < pcaConf.numDev; i++) {
      pcaConf.pcaDev[i].pNow    = 0;
      pcaConf.pcaDev[i].pTtl    = 0;
      pcaConf.pcaDev[i].nextTX  = 0;
      pcaConf.pcaDev[i].retries = 0;
    }
    return 1;
  } else {
    // invalid crc
    return 0;
  }
}

// save config to EEPROM
static void saveConf() {
  uint16_t len = sizeof(pcaConf);
  byte *pPtrByte = (byte*)&pcaConf;        // byte Ptr to pcaConf

  eeprom_crc = 0;
  for (int i=0; i < (len - 2); i++) {
    eeprom_crc = crc16_pca301_update(eeprom_crc, *pPtrByte);
    pPtrByte++;
  }
  pcaConf.crc = eeprom_crc;
  
  eeprom_write_block(&pcaConf, (void *) 0, len);
}

// erase config
static void eraseConf() {
  pcaConf.numDev = 0;
}

//- fill config ------------------------------------------------------------------------------------
static void fillConf() {
  pcaConf.numDev     = 0;
  pcaConf.pollIntv   = 300;                             // default poll interval in 1/10th seconds
  pcaConf.deadIntv   = 3000;                            // dead device poll retry interval in 1/10th seconds
  pcaConf.quiet      = 1;                               // quiet, 1=suppress TX and bad packets
  
  pcaConf.pcaDev[0]  = (struct_pcaDev){1 ,0xAAAAA};    // device 1
  pcaConf.pcaDev[1]  = (struct_pcaDev){2 ,0xBBBBB};    // device 2
}


static uint16_t crc16_pca301_update(uint16_t crc, uint8_t data) {
  int i;
  crc = crc ^ ((uint16_t)data << 8);
  for (i=0; i<8; i++) {
    if (crc & 0x8000)
      crc = (crc << 1) ^ 0x8005;
    else
      crc <<= 1;
  }
  return crc;
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// E N D
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
