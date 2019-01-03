/**********************************
   SMBus FOR 3DR SOLO with Arduino UNO R3
   STAVROPOULOS
   Code Version 0.02 beta

   MUCH OF THIS CODE WAS COPIED FROM
   https://github.com/PowerCartel/PackProbe/blob/master/PackProbe/PackProbe.ino
   https://github.com/ArduPilot/PX4Firmware/blob/master/src/drivers/batt_smbus/batt_smbus.cpp

 **********************************/

/**********************************
   Configured for Arduino UNO R3
   you will need to use external pull up resistors of
   4.7k-ohm to pull the SDA and SCL lines up to 3.3v
 **********************************/


/**********************************
   CONFIGURE I2C/SERIAL ON ARDUINO
 **********************************/

//DEFINE SDA AND SCL PINS
#define SCL_PIN 5                 //COMMUNICATION PIN 5 ON MEGA
#define SCL_PORT PORTD

#define SDA_PIN 4                 //COMMUNICATION PIN 6 ON MEGA
#define SDA_PORT PORTD


//CONFIGURE I2C MODES
#define I2C_TIMEOUT 100           //PREVENT SLAVE DEVICES FROM STRETCHING LOW PERIOD OF THE CLOCK INDEFINITELY AND LOCKING UP MCU BY DEFINING TIMEOUT
//#define I2C_NOINTERRUPT 1       //SET TO 1 IF SMBus DEVICE CAN TIMEOUT
//#define I2C_FASTMODE 1          //THE STANDARD I2C FREQ IS 100kHz.  USE THIS TO PERMIT FASTER UP TO 400kHz.
#define I2C_SLOWMODE 1            //THE STANDARD I2C FREQ IS 100kHz.  USE THIS TO PERMIT SLOWER, DOWN TO 25kHz.


#define BAUD_RATE 115200
#include <SoftI2CMaster.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(A5, A4, A3, A2, A1, A0);
/**********************************
   CONFIGURE SERIAL LIBRARY
 **********************************/
#include <Wire.h>
#include <mcp_can.h>
#include <SPI.h>


/**********************************
   DEFINE VARIABLES AND SMBus MAPPINGS
 **********************************/
#define BATT_SMBUS_ADDR                     0x0B                ///< I2C address
#define BATT_SMBUS_ADDR_MIN                 0x08                ///< lowest possible address
#define BATT_SMBUS_ADDR_MAX                 0x7F                ///< highest possible address

//BUS MAPPINGS FROM DEV.3DR
#define BATT_SMBUS_TEMP                     0x08                ///< temperature register
#define BATT_SMBUS_VOLTAGE                  0x09                ///< voltage register
#define BATT_SMBUS_REMAINING_CAPACITY       0x0f                ///< predicted remaining battery capacity as a percentage
#define BATT_SMBUS_FULL_CHARGE_CAPACITY     0x10                ///< capacity when fully charged
#define BATT_SMBUS_DESIGN_CAPACITY          0x18                ///< design capacity register
#define BATT_SMBUS_DESIGN_VOLTAGE           0x19                ///< design voltage register
#define BATT_SMBUS_SERIALNUM                0x1c                ///< serial number register
#define BATT_SMBUS_MANUFACTURE_NAME         0x20                ///< manufacturer name
#define BATT_SMBUS_MANUFACTURE_DATA         0x23                ///< manufacturer data
#define BATT_SMBUS_MANUFACTURE_INFO         0x25                ///< cell voltage register
#define BATT_SMBUS_CURRENT                  0x2a                ///< current register
#define BATT_SMBUS_MEASUREMENT_INTERVAL_US  (1000000 / 10)      ///< time in microseconds, measure at 10hz
#define BATT_SMBUS_TIMEOUT_US               10000000            ///< timeout looking for battery 10seconds after startup
#define BATT_SMBUS_BUTTON_DEBOUNCE_MS       300                 ///< button holds longer than this time will cause a power off event

#define BATT_SMBUS_PEC_POLYNOMIAL           0x07                ///< Polynomial for calculating PEC
#define BATT_SMBUS_I2C_BUS                  PX4_I2C_BUS_EXPANSION


//BUS MAPPINGS FROM SMBus PROTOCOL DOCUMENTATION
#define BATTERY_MODE             0x03
#define CURRENT                  0x0A
#define RELATIVE_SOC             0x0D
#define ABSOLUTE_SOC             0x0E
#define TIME_TO_FULL             0x13
#define CHARGING_CURRENT         0x14
#define CHARGING_VOLTAGE         0x15
#define BATTERY_STATUS           0x16
#define CYCLE_COUNT              0x17
#define SPEC_INFO                0x1A
#define MFG_DATE                 0x1B
#define DEV_NAME                 0x21   // String
#define CELL_CHEM                0x22   // String
#define CELL4_VOLTAGE            0x3C   // Indidual cell voltages don't work on Lenovo and Dell Packs
#define CELL3_VOLTAGE            0x3D
#define CELL2_VOLTAGE            0x3E
#define CELL1_VOLTAGE            0x3F
#define STATE_OF_HEALTH          0x4F
//END BUS MAPPINGS

#define bufferLen 32
uint8_t i2cBuffer[bufferLen];

// standard I2C address for Smart Battery packs
byte deviceAddress = BATT_SMBUS_ADDR;

const int SPI_CS_PIN = 10;  // This is the Chip select Line for the CAN Module
MCP_CAN CAN(SPI_CS_PIN);

void setup()
{
  // Turn ON message
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Ultralife");
  lcd.setCursor(0, 1);
  lcd.print("Batteries");
  delay(2000);
  lcd.clear();


  //INITIATE SERIAL CONSOLE
  Serial.begin(BAUD_RATE);
  Serial.println(i2c_init());

  //SETUP I2C INPUT PINS
  //pinMode(27,INPUT_PULLUP);                             //use external pull up resistor instead
  //pinMode(28,INPUT_PULLUP);                             //use external pull up resistor instead

  Serial.flush();

  while (!Serial) {
    ;                                                       //wait for Console port to connect.
  }

  Serial.println("Console Initialized");

  i2c_init();                                             //i2c_start initialized the I2C system.  will return false if bus is locked.
  Serial.println("I2C Inialized");
  scan();



  while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Module again");
    delay(100);
  }
  Serial.println("CAN BUS Module init ok!");


}

String converter(uint8_t *str)
{
  return String((char *)str);
}
int count_digit(int n)
{
  int count = 0;

  while (n != 0)
  {
    n /= 10;
    ++count;
  }
  return count;
}


void parse_send(int cell_voltages[], int arr_size)
{
  //Serial.println("in CAN Method");

  int count = 0;
  
  unsigned char data[arr_size+1] = {0};
  data[0] = 0xc;
  count = count_digit(cell_voltages[1]);
  
 for (int i = 0 ; i <=arr_size-1; i++)
  {    
    int  n = cell_voltages[i]; 
    for (int j = count; j >0; j--)
    {
      data[j] = n % 10;
      n /= 10;
    }
    CAN.sendMsgBuf(0x00, 0, arr_size+1 , data);
    delay(100);
  }
  
  //return;
}

int send_digits_can(int identifier, int data)
{
  int digits = count_digit(data);
  unsigned char data_can[digits+1] = {0};
  data_can[0] = identifier;
  for (int j = digits; j >0; j--)
    {
      data_can[j] = data % 10;
      data /= 10;
    }
    CAN.sendMsgBuf(0x00, 0, digits+1 , data_can);
    delay(100);
  
}

int send_string_can(int identifier, String data)
{
  data.trim();
  data.replace(" ","");
  int len = data.length();
  Serial.println(len);
  unsigned char data_can[len+1] = {0};
  data_can[0] = identifier;
  for(int i=0;i<=len;i++)
  {
    Serial.println(data[i]);
    data_can[i+1] = data[i];
  }
  CAN.sendMsgBuf(0x00, 0, len+1 , data_can);
  delay(100);
}


int fetchWord(byte func)
{
  i2c_start(deviceAddress << 1 | I2C_WRITE);              //Initiates a transfer to the slave device with the (8-bit) I2C address addr.
  //Alternatively, use i2c_start_wait which tries repeatedly to start transfer until acknowledgment received
  //i2c_start_wait(deviceAddress<<1 | I2C_WRITE);
  i2c_write(func);                                        //Sends a byte to the previously addressed device. Returns true if the device replies with an ACK.
  i2c_rep_start(deviceAddress << 1 | I2C_READ);           //Sends a repeated start condition, i.e., it starts a new transfer without sending first a stop condition.
  byte b1 = i2c_read(false);                              //i2c_read Requests to receive a byte from the slave device. If last is true,
  //then a NAK is sent after receiving the byte finishing the read transfer sequence.
  byte b2 = i2c_read(true);
  i2c_stop();                                             //Sends a stop condition and thereby releases the bus.
  return (int)b1 | ((( int)b2) << 8);
}

uint8_t i2c_smbus_read_block ( uint8_t command, uint8_t* blockBuffer, uint8_t blockBufferLen )
{
  uint8_t x, num_bytes;
  i2c_start((deviceAddress << 1) + I2C_WRITE);
  i2c_write(command);
  i2c_rep_start((deviceAddress << 1) + I2C_READ);
  num_bytes = i2c_read(false);                              //num of bytes; 1 byte will be index 0
  num_bytes = constrain(num_bytes, 0, blockBufferLen - 2);  //room for null at the end
  for (x = 0; x < num_bytes - 1; x++) {                     //-1 because x=num_bytes-1 if x<y; last byte needs to be "nack"'d, x<y-1
    blockBuffer[x] = i2c_read(false);
  }
  blockBuffer[x++] = i2c_read(true);                        //this will nack the last byte and store it in x's num_bytes-1 address.
  blockBuffer[x] = 0;                                       // and null it at last_byte+1
  i2c_stop();
  return num_bytes;
}

void scan()
{
  byte i = 0;
  for ( i = 0; i < 127; i++  )
  {
    //Serial.print("Address: 0x");   //print to know the address.
    //Serial.print(i,HEX);
    bool ack = i2c_start(i << 1 | I2C_WRITE);
    if ( ack ) {
      //Serial.println(": OK");
      //Serial.flush();
    }
    else {
      //Serial.println(": -");
      //Serial.flush();
    }
    i2c_stop();
  }
}

void loop()
{
  uint8_t length_read = 0;


  int dsgn_capacity = fetchWord(BATT_SMBUS_DESIGN_CAPACITY);
  send_digits_can(176, dsgn_capacity);   //B0 - 176
  
  int dsgn_voltage = fetchWord(BATT_SMBUS_DESIGN_VOLTAGE);
  send_digits_can(177, dsgn_voltage);  //B1 - 177

  int chrg_current = fetchWord(CHARGING_CURRENT);
  send_digits_can(178, chrg_current);  //B2 - 178

  int chrg_voltage = fetchWord(CHARGING_VOLTAGE);
  send_digits_can(179, chrg_voltage);  //B3 - 179
  
  int sl_no = fetchWord(BATT_SMBUS_SERIALNUM);
  
  float total_voltage = (float)fetchWord(BATT_SMBUS_VOLTAGE) / 1000;
  send_digits_can(208, (int)(total_voltage*100.0));  //D0 
  int full_capacity = fetchWord(BATT_SMBUS_FULL_CHARGE_CAPACITY);
  send_digits_can(209, full_capacity);      //D1
  unsigned int tempk = fetchWord(BATT_SMBUS_TEMP);
  float tempC = (float)tempk / 10.0 - 273.15;
  send_digits_can(210, tempC);   //D2
  
  
  int cycle_cnt = fetchWord(CYCLE_COUNT);
  int rel_charge = fetchWord(RELATIVE_SOC); 
  int abs_charge = fetchWord(ABSOLUTE_SOC);
  
  int current = fetchWord(CURRENT);

  int spec_info = fetchWord(SPEC_INFO);

  int cell1 = fetchWord(CELL1_VOLTAGE);
  int cell2 = fetchWord(CELL2_VOLTAGE);
  int cell3 = fetchWord(CELL3_VOLTAGE);
  int cell4 = fetchWord(CELL4_VOLTAGE);

  /////////////////////////  SEND CEll VOLTAGES ON CAN BUS   ///////////////////////////////////
  int cell_voltages[4] = {cell1, cell2, cell3, cell4};
  int len = sizeof(cell_voltages) / sizeof(cell_voltages[0]);
  parse_send(cell_voltages, len);

  ////////////////////////////


  Serial.println("****START*********************************************************************************************************");

  Serial.print("Manufacturer Name: ");
  length_read = i2c_smbus_read_block(BATT_SMBUS_MANUFACTURE_NAME, i2cBuffer, bufferLen);
  String manf_name = converter(i2cBuffer);
  send_string_can(160, manf_name);
  Serial.write(i2cBuffer, length_read);
  Serial.println("");

  Serial.print("Device Name: ");
  length_read = i2c_smbus_read_block(DEV_NAME, i2cBuffer, bufferLen);
  Serial.write(i2cBuffer, length_read);
  String device_name = converter(i2cBuffer);
  send_string_can(161, device_name);
  Serial.println("");

  Serial.print("Chemistry: ");
  length_read = i2c_smbus_read_block(CELL_CHEM, i2cBuffer, bufferLen);
  Serial.write(i2cBuffer, length_read);
  String chemistry = converter(i2cBuffer);
  send_string_can(162, chemistry);
  Serial.println("");

  send_digits_can(163,sl_no);
  
  Serial.print("Design Capacity: " );
  Serial.println(dsgn_capacity);

  Serial.print("Design Voltage: " );
  Serial.println(dsgn_voltage);

  Serial.print("Serial Number: ");
  Serial.println(sl_no);

  Serial.print("Voltage: ");
  Serial.println(total_voltage);

  Serial.print("Full Charge Capacity: " );
  Serial.println(full_capacity);

  Serial.print("Remaining Capacity: " );
  Serial.println(fetchWord(BATT_SMBUS_REMAINING_CAPACITY));

  Serial.print("Temp: ");
  Serial.println(tempC);

  Serial.print("Current (mA): " );
  Serial.println(current);

  String formatted_date = "Manufacture Date (Y-M-D): ";
  int mdate = fetchWord(MFG_DATE);
  int mday = B00011111 & mdate;
  int mmonth = mdate >> 5 & B00001111;
  int myear = 1980 + (mdate >> 9 & B01111111);
  formatted_date += myear;
  formatted_date += "-";
  formatted_date += mmonth;
  formatted_date += "-";
  formatted_date += mday;
  Serial.println(formatted_date);

  Serial.print("Specification Info: ");
  Serial.println(spec_info);

  Serial.print("Cycle Count: " );
  Serial.println(cycle_cnt);

  Serial.print("Relative Charge(%): ");
  Serial.println(rel_charge);

  Serial.print("Absolute Charge(%): ");
  Serial.println(abs_charge);

  Serial.print("Minutes remaining for full charge: ");
  Serial.println(fetchWord(TIME_TO_FULL));


  Serial.print("Cell 1 Voltage: ");
  Serial.println(cell1);

  Serial.print("Cell 2 Voltage: ");
  Serial.println(cell2);

  Serial.print("Cell 3 Voltage: ");
  Serial.println(cell3);

  Serial.print("Cell 4 Voltage: ");
  Serial.println(cell4);

  Serial.print("State of Health: ");
  Serial.println(fetchWord(STATE_OF_HEALTH));

  //    Serial.print("Battery Mode (BIN): 0b");
  //    Serial.println(fetchWord(BATTERY_MODE),BIN);

  //    Serial.print("Battery Status (BIN): 0b");
  //    Serial.println(fetchWord(BATTERY_STATUS),BIN);

  Serial.print("Charging Current: ");
  Serial.println(chrg_current);

  Serial.print("Charging Voltage: ");
  Serial.println(chrg_voltage);

  Serial.print("Current (mA): " );
  Serial.println(fetchWord(CURRENT));

  Serial.println("****STOP****");
  //delay(5000);

  //region Region "Lcd Printing"

  lcd.setCursor(0, 0); lcd.print("Voltage(V):"); lcd.setCursor(11, 0); lcd.print(total_voltage);
  lcd.setCursor(0, 1); lcd.print("Current(mA):"); lcd.setCursor(12, 1); lcd.print(current);
  delay(3000);
  lcd.clear();

  lcd.setCursor(0, 0); lcd.print("Cell Voltages"); lcd.setCursor(0, 1); lcd.print("(mV)"); delay(2000); lcd.clear();
  lcd.setCursor(0, 0); lcd.print("C1:"); lcd.setCursor(3, 0); lcd.print(cell1); lcd.setCursor(7, 0); lcd.print("  C2:"); lcd.setCursor(12, 0); lcd.print(cell2);
  lcd.setCursor(0, 1); lcd.print("C3:"); lcd.setCursor(3, 1); lcd.print(cell3); lcd.setCursor(7, 1); lcd.print("  C4:"); lcd.setCursor(12, 1); lcd.print(cell4);
  delay(6000);
  lcd.clear();

  //endregion



}


