#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#include <ArduinoJson.h>

#include <EEPROM.h>

#include <ModbusMaster.h>
#include "REG_CONFIG.h"
#include <HardwareSerial.h>
#include "HardwareSerial_NB_BC95.h"


#include <TaskScheduler.h>

#define _TASK_TIMECRITICAL

HardwareSerial modbus(2);
HardwareSerial_NB_BC95 AISnb;

BluetoothSerial SerialBT;

// Global copy of slave
#define NUMSLAVES 10
int SlaveCnt = 0;

#define CHANNEL 3
#define PRINTSCANRESULTS 0

String deviceToken = "WQ5aSKt6tra0hCewevKN";
String serverIP = "103.27.203.83"; // Your Server IP;
String serverPort = "19956"; // Your Server Port;
String json = "";

ModbusMaster node;
void t1CallgetMeter();
void t2CallsendViaNBIOT();
void t4Restart();
//TASK
Task t1(10000, TASK_FOREVER, &t1CallgetMeter);
Task t2(15000, TASK_FOREVER, &t2CallsendViaNBIOT);
Task t4(3600000, TASK_FOREVER, &t4Restart);

Scheduler runner;
String _config = "{\"_type\":\"retrattr\",\"Tn\":\"8966031940014308682\",\"keys\":[\"epoch\",\"ip\"]}";
unsigned long _epoch = 0;
String _IP = "";
String dataJson = "";
boolean validEpoc = false;

StaticJsonDocument<400> doc;

struct Meter
{
  String cA;
  String cB;
  String cC;
  String cN;
  float cG;
  String cAvg;
  String vAB;
  String vBC;
  String vCA;
  String vAN;
  String vBN;
  String vCN;

  String vLLAvg;
  String vLNAvg;
  String freq;
  String apTotal;

  ////////////////////////
  String cuA;  // Current Unbalance A
  String cuB;  // Current Unbalance B
  String cuC;  // Current Unbalance C
  String cuW;  // Current Unbalance Worst

  String vuAB;
  String vuBC;
  String vuCA;
  String vuLLW; // Voltage Unbalance L-L Worst

  String vuAN;
  String vuBN;
  String vuCN;
  String vuLNW; // Voltage Unbalance L-N Worst

  String apA;
  String apB;
  String apC;
  String apT; // Active Power Total
  String rpA;
  String rpB;
  String rpC;
  String rpT; // Reactive Power Total
  String appA;
  String appB; 
  String appC;
  String appT; // Apparent Power Total
  
  ////////////////////////

  String di1;
  String di2;
  String di3;
  String di4;

  String Temp1;
  String Temp2;

  String sdmVolt;
  String sdmCurrent;
  
};
Meter meter;
signal meta;



void _writeEEPROM(String data) {
  Serial.print("Writing Data:");
  Serial.println(data);

  writeString(10, data);  //Address 10 and String type data
  delay(10);
}

void _loadConfig() {
  serverIP = read_String(10);
  Serial.print("IP:");
  Serial.println(serverIP);
}


char  char_to_byte(char c)
{
  if ((c >= '0') && (c <= '9'))
  {
    return (c - 0x30);
  }
  if ((c >= 'A') && (c <= 'F'))
  {
    return (c - 55);
  }
}
void _init() {

  Serial.println(_config);

  do {
    UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, _config);
    dataJson = "";
    deviceToken = AISnb.getNCCID();
    Serial.print("nccid:");
    Serial.println(deviceToken);


    UDPReceive resp = AISnb.waitResponse();
    AISnb.receive_UDP(resp);
    Serial.print("waitData:");
    Serial.println(resp.data);


    for (int x = 0; x < resp.data.length(); x += 2)
    {
      char c =  char_to_byte(resp.data[x]) << 4 | char_to_byte(resp.data[x + 1]);

      dataJson += c;
    }
    Serial.println(dataJson);
    DeserializationError error = deserializeJson(doc, dataJson);

    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      validEpoc = true;
      delay(4000);
    } else {
      validEpoc = false;
      unsigned long epoch = doc["epoch"];
      _epoch = epoch;
      String ip = doc["ip"];
      _IP = ip;
       Serial.println(dataJson);
      Serial.print("epoch:");  Serial.println(_epoch);
      _writeEEPROM(_IP);
      Serial.println(_IP);

    }

  } while (validEpoc);


}


void writeString(char add, String data)
{
  int _size = data.length();
  int i;
  for (i = 0; i < _size; i++)
  {
    EEPROM.write(add + i, data[i]);
  }
  EEPROM.write(add + _size, '\0'); //Add termination null character for String Data
  EEPROM.commit();
}


String read_String(char add)
{
  int i;
  char data[100]; //Max 100 Bytes
  int len = 0;
  unsigned char k;
  k = EEPROM.read(add);
  while (k != '\0' && len < 500) //Read until null character
  {
    k = EEPROM.read(add + len);
    data[len] = k;
    len++;
  }
  data[len] = '\0';

  return String(data);
}

void setup()
{

  Serial.begin(115200);
  SerialBT.begin("VaccienMonitor"); //Bluetooth device name
  SerialBT.println("VaccienMonitor");
  modbus.begin(9600, SERIAL_8N1, 16, 17);


  Serial.println();
  Serial.println(F("***********************************"));


  runner.init();
  Serial.println("Initialized scheduler");

  runner.addTask(t1);
  Serial.println("added t1");
  runner.addTask(t2);
  Serial.println("added t2");

  runner.addTask(t4);
  Serial.println("added t4");
  delay(2000);
  t1.enable();  Serial.println("Enabled t1");
  t2.enable();  Serial.println("Enabled t2");
  t4.enable();  Serial.println("Enabled t4");

  AISnb.debug = true;
  AISnb.setupDevice(serverPort);
  _init();
  _loadConfig();
  String ip1 = AISnb.getDeviceIP();

  

}

void t2CallsendViaNBIOT ()
{
  meta = AISnb.getSignal();

  Serial.print("RSSI:"); Serial.println(meta.rssi);

  json = "";
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);
  json.concat("\",\"cA\":");
  json.concat(meter.cA);
  json.concat(",\"cB\":");
  json.concat(meter.cB);
  json.concat(",\"cC\":");
  json.concat(meter.cC);
  json.concat(",\"cN\":");
  json.concat(meter.cN);
  json.concat(",\"cG\":");
  if (isnan(meter.cG))
  {
    json.concat(0);
  }
  else
  {
    json.concat(String(meter.cG));
  }
  json.concat(",\"cAvg\":");
  json.concat(meter.cAvg);
  json.concat(",\"vAB\":");
  json.concat(meter.vAB);
  json.concat(",\"vBC\":");
  json.concat(meter.vBC);
  json.concat(",\"vCA\":");
  json.concat(meter.vCA);
  json.concat(",\"vAN\":");
  json.concat(meter.vAN);
  json.concat(",\"vBN\":");
  json.concat(meter.vBN);
  json.concat(",\"vCN\":");
  json.concat(meter.vCN);
  json.concat(",\"freq\":");
  json.concat(meter.freq);
  json.concat(",\"apTotal\":");
  json.concat(meter.apTotal);
  json.concat(",\"t1\":");
  json.concat(meter.Temp1);
  json.concat(",\"t2\":");
  json.concat(meter.Temp2);
  json.concat(",\"sdmVolt\":");
  json.concat(meter.sdmVolt);
  json.concat(",\"d2\":");
  json.concat(meter.sdmCurrent);
  json.concat(",\"rssi\":");
  json.concat(meta.rssi);
  json.concat(",\"csq\":");
  json.concat(meta.csq);
  json.concat("}");
  Serial.println(json);
  //  SerialBT.println(json);
  //
  UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);
  UDPReceive resp = AISnb.waitResponse();
  Serial.print("rssi:");
  Serial.println(meta.rssi);
  //  SerialBT.print("rssi:");
  //  SerialBT.println(meta.rssi);
  
}
 
void readMeter()
{
  meter.cA = read_Modbus(c_A);
  meter.cB = read_Modbus(c_B);
  meter.cC = read_Modbus(c_C);
  meter.cN = read_Modbus(c_N);
  meter.cG = read_Modbus(c_G);
  meter.cAvg = read_Modbus(c_Avg);

  meter.vAB = read_Modbus(v_A_B);
  meter.vBC = read_Modbus(v_B_C);
  meter.vCA = read_Modbus(v_C_A);
  meter.vLLAvg = read_Modbus(v_LL_Avg);
  meter.vAN = read_Modbus(v_A_N);
  meter.vBN = read_Modbus(v_B_N);
  meter.vCN = read_Modbus(v_C_N);
  meter.vLNAvg = read_Modbus(v_LN_Avg);
  meter.apTotal = read_Modbus(ap_Total);
  meter.freq = read_Modbus(v_Freq);

  //
  meter.cuA = read_Modbus(cu_A);  // Current Unbalance A
  meter.cuB = read_Modbus(cu_B);  // Current Unbalance B
  meter.cuC = read_Modbus(cu_C);  // Current Unbalance C
  meter.cuW = read_Modbus(cu_W);  // Current Unbalance Worst

  meter.vuAB = read_Modbus(vu_AB);
  meter.vuBC = read_Modbus(vu_BC);
  meter.vuCA = read_Modbus(vu_CA);
  meter.vuLLW = read_Modbus(vu_LLW); // Voltage Unbalance L-L Worst

  meter.vuAN = read_Modbus(vu_AN);
  meter.vuBN = read_Modbus(vu_BN);
  meter.vuCN = read_Modbus(vu_CN);
  meter.vuLNW = read_Modbus(vu_LNW); // Voltage Unbalance L-N Worst

  meter.apA = read_Modbus(ap_A);
  meter.apB = read_Modbus(ap_B);
  meter.apC = read_Modbus(ap_C);
  meter.apT = read_Modbus(ap_T); // Active Power Total
  meter.rpA = read_Modbus(rp_A);
  meter.rpB = read_Modbus(rp_B);
  meter.rpC = read_Modbus(rp_C);
  meter.rpT = read_Modbus(rp_T); // Reactive Power Total
  meter.appA = read_Modbus(app_A);
  meter.appB = read_Modbus(app_B); 
  meter.appC = read_Modbus(app_C);
  meter.appT = read_Modbus(app_T); // Apparent Power Total
  //
  
  meter.sdmVolt = read_Modbus_1Byte(ID_SDM, SDM120_Volt);
  meter.sdmCurrent = read_Modbus_1Byte(ID_SDM, SDM120_Current);

  meter.Temp1 = read_Modbus_1Byte(ID_Temp1, pvTemp);
  meter.Temp2 = read_Modbus_1Byte(ID_Temp2, pvTemp);

  
  Serial.print("Current A: ");
  Serial.print(meter.cA);
  Serial.println(" Amp");
  Serial.print("Current B: ");
  Serial.print(meter.cB);
  Serial.println(" Amp");
  Serial.print("Current C: ");
  Serial.print(meter.cC);
  Serial.println(" Amp");
  Serial.print("Current N: ");
  Serial.print(meter.cN);
  Serial.println(" Amp");
  Serial.print("Current G: ");
  Serial.print(meter.cG);
  Serial.println(" Amp");
  Serial.print("Current Avg: ");
  Serial.print(meter.cAvg);
  Serial.println(" Amp");

  Serial.print("Voltage A-B: ");
  Serial.print(meter.vAB);
  Serial.println(" Volt");
  Serial.print("Voltage B-C: ");
  Serial.print(meter.vBC);
  Serial.println(" Volt");
  Serial.print("Voltage C-A: ");
  Serial.print(meter.vCA);
  Serial.println(" Volt");
  Serial.print("Voltage L-L Avg: ");
  Serial.print(meter.vLLAvg);
  Serial.println(" Volt");
  Serial.print("Voltage A-N: ");
  Serial.print(meter.vAN );
  Serial.println(" Volt");
  Serial.print("Voltage B-N: ");
  Serial.print(meter.vBN );
  Serial.println(" Volt");
  Serial.print("Voltage C-N: ");
  Serial.print(meter.vCN );
  Serial.println(" Volt");

  Serial.print("Volt AVG: ");
  Serial.print(meter.vLNAvg);
  Serial.println(" Volt");
  Serial.print("Frequency: ");
  Serial.print(read_Modbus(v_Freq));
  Serial.println(" Hz");

  Serial.print("Active Power Total: ");
  Serial.print(read_Modbus(ap_Total));
  Serial.println(" Kw");

  Serial.print("Temp1: ");
  Serial.print(meter.Temp1);
  Serial.println(" c");
  Serial.print("Temp2: ");
  Serial.print(meter.Temp2);
  Serial.println(" c");
  Serial.print("SDM.Volt: ");
  Serial.print(meter.sdmVolt);
  Serial.print("SDM.Current: ");
  Serial.print(meter.sdmCurrent);
  
  
  Serial.println("");
  Serial.println("");
  delay(500);
}

void t1CallgetMeter() {     // Update read all data
  readMeter();
}

void t4Restart() {     // Update read all data
  Serial.println("Restart");
  ESP.restart();
}
float HexTofloat(uint32_t x)
{
  return (*(float*)&x);
}


float read_Modbus(uint16_t  REG)
{
  static uint32_t i;
  uint8_t j, result;
  uint16_t data[2];
  uint32_t value = 0;
  float val = 0.0;

  // communicate with Modbus slave ID 1 over Serial (port 2)
  node.begin(ID_PowerMeter, modbus);

  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readHoldingRegisters(REG, 2);

  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 2; j++)
    {
      data[j] = node.getResponseBuffer(j);
    }
    value = data[0];
    value = value << 16;
    value = value + data[1];

    val = HexTofloat(value);

    return val;
  } else {
    Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug
    //    delay(1000);
    return 0;
  }
}

int read_Modbus_1Byte(char addr, uint16_t  REG)
{
  static uint32_t i;
  uint8_t j, result;
  uint16_t data[2];
  uint32_t value = 0;
  float val = 0.0;

  // communicate with Modbus slave ID 1 over Serial (port 2)
  node.begin(addr, modbus);

  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readHoldingRegisters(REG, 1);

  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 1; j++)
    {
      data[j] = node.getResponseBuffer(j);
    }
    value = data[0];

    return value;
  } else {
    Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug
    //    delay(1000);
    return 0;
  }
}

void loop()
{
  runner.execute();
}
