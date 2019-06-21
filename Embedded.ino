#include <WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>
#include "Adafruit_SGP30.h"
#include <SoftwareSerial.h>
#include "font.h"

const char *ssid = "air";
const char *password = "airairair";
// 接收路由器的ID和密码
WiFiUDP Udp;
unsigned int localUdpPort = 2333;
// 向服务器发送传感器信息
WiFiUDP dataUdp;
unsigned int dataUdpPort = 23333;

const char *new_ssid;
const char *new_password;
char new_id[50];
char new_pw[50];
boolean connectedAPP = true;
boolean connectedRoute = true;

const char *serverip = "192.168.43.224";
const unsigned int sendport = 23333;

#define MINUTE (60000UL)
// =====================传感器变量定义==========================
// PM==========================================================
SoftwareSerial PMSerial(26, 27);
unsigned char PMRev_buf[20], PMCounter = 0;
unsigned PMSign = 0;
float PM2_5_TEMP = 0.0;
float PM10_TEMP = 0.0;

// GYMCU680====================================================
SoftwareSerial GYMCU680Serial(16, 17);
unsigned char GYMCU680Rev_buf[30], GYMCU680Counter = 0;
unsigned GYMCU680Sign = 0;
uint16_t temp1 = 0;
int16_t temp2 = 0;
float Temperature_TEMP, Humidity_TEMP;
uint32_t IAQ_TEMP;

// MQ-4========================================================
int limit;
int CH4;
const int AOUTpin = 35;
const int DOUTpin = 34;

// SGP30=======================================================
Adafruit_SGP30 sgp;
#define SDA_SGP 19
#define SCL_SGP 23
TwoWire WireSGP = TwoWire(1);

/* return absolute humidity [mg/m^3] with approximation formula
* @param temperature [°C]
* @param humidity [%RH]
*/
uint32_t getAbsoluteHumidity(float temperature, float humidity)
{
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                // [mg/m^3]
  return absoluteHumidityScaled;
}

int counter = 0;
// ==========================END===============================

// ======================== OLED===============================
int scl = 22; //定义数字接口22
int sda = 21; //定义数字接口21

#define OLED_SCLK_Clr() digitalWrite(scl, LOW) //SCL
#define OLED_SCLK_Set() digitalWrite(scl, HIGH)

#define OLED_SDIN_Clr() digitalWrite(sda, LOW) //SDA
#define OLED_SDIN_Set() digitalWrite(sda, HIGH)

#define OLED_CMD 0  //写命令
#define OLED_DATA 1 //写数据

uint8_t OLED_GRAM[128][8]; //将要显示的缓存内容
#define u32 uint32_t
// ============================END=============================

void setup()
{
  // 传感器、OLED、WiFi初始化

  // log serial================================================
  Serial.begin(115200);
  delay(10);

  // OLED======================================================
  OLED_Init();
  OLED_ColorTurn(0);   //0正常显示 1反色显示
  OLED_DisplayTurn(0); //0正常显示 1翻转180度显示

  // 传感器初始化
  // setup SGP30===============================================
  WireSGP.begin(SDA_SGP, SCL_SGP, 100000);
  if (!sgp.begin(&WireSGP))
  {
    Serial.println("Sensor not found :(");
    while (1)
      ;
  }
  Serial.print("Found SGP30 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);

  // PM========================================================
  PMSerial.begin(9600);
  PMSerial.listen();
  delay(5000);

  // SDS011 主动上报模式
  PMSerial.write((uint8_t)0XC0);
  delay(100);
  PMSerial.write((uint8_t)0X06);
  delay(100);
  PMSerial.write((uint8_t)0X00);
  delay(100);
  PMSerial.write((uint8_t)0XB1);
  delay(100);
  PMSerial.write((uint8_t)0X00);
  delay(100);
  PMSerial.write((uint8_t)0X00);
  delay(100);
  PMSerial.write((uint8_t)0XC9);
  delay(100);
  PMSerial.write((uint8_t)0X3C);
  delay(100);

  // GY_MCU680=================================================
  GYMCU680Serial.begin(9600);
  GYMCU680Serial.listen();
  delay(4000);

  GYMCU680Serial.write(0XA5);
  GYMCU680Serial.write(0X55);
  GYMCU680Serial.write(0X3F);
  GYMCU680Serial.write(0X39);
  delay(100);

  GYMCU680Serial.write(0XA5);
  GYMCU680Serial.write(0X56);
  GYMCU680Serial.write(0X02);
  GYMCU680Serial.write(0XFD);
  delay(100);

  // MQ-4=======================================================
  pinMode(DOUTpin, INPUT);

  // 连接手机APP
  connectAPP();
  Udp.begin(localUdpPort);
  delay(500);

  // 连接路由器
  connectRoute();
  delay(500);
}

String S_Temperature;
String S_Humidity;
String S_CH4;
String S_IAQ;
String S_PM2_5;
String S_PM10;
String S_eCO2;
String S_TVOC;

void loop()
{
  // 获取传感器信息
  String DATA = getData();
  Serial.println(DATA);

  // 发送传感器信息到服务器
  sendData(DATA);

  char c_Temperature[S_Temperature.length() + 1];
  char c_Humidity[S_Humidity.length() + 1];
  char c_CH4[S_CH4.length() + 1];
  char c_PM2_5[S_PM2_5.length() + 1];
  char c_PM10[S_PM10.length() + 1];
  char c_eCO2[S_eCO2.length() + 1];
  char c_TVOC[S_TVOC.length() + 1];
  char c_IAQ[S_IAQ.length() + 1];

  S_Temperature.toCharArray(c_Temperature, S_Temperature.length() + 1);
  S_Humidity.toCharArray(c_Humidity, S_Humidity.length() + 1);
  S_CH4.toCharArray(c_CH4, S_CH4.length() + 1);
  S_IAQ.toCharArray(c_IAQ, S_IAQ.length() + 1);
  S_PM2_5.toCharArray(c_PM2_5, S_PM2_5.length() + 1);
  S_PM10.toCharArray(c_PM10, S_PM10.length() + 1);
  S_eCO2.toCharArray(c_eCO2, S_eCO2.length() + 1);
  S_TVOC.toCharArray(c_TVOC, S_TVOC.length() + 1);

  // OLED显示
  OLED_Clear();
  OLED_ShowString(0, 1, c_Temperature, 12);
  OLED_ShowString(0, 17, c_Humidity, 12);
  OLED_ShowString(0, 33, c_CH4, 12);
  OLED_ShowString(0, 49, c_IAQ, 12);
  OLED_Refresh();
  delay(3000);
  OLED_Clear();
  OLED_ShowString(0, 1, c_PM2_5, 12);
  OLED_ShowString(0, 17, c_PM10, 12);
  OLED_ShowString(0, 33, c_eCO2, 12);
  OLED_ShowString(0, 49, c_TVOC, 12);
  OLED_Refresh();
  delay(3000);
}

String getData()
{
  // 获取传感器信息
  // PM2.5 && PM10=============================================
  unsigned char PM_i = 0;
  unsigned char PMChecksum = 0;
  float PM2_5 = 0.0;
  float PM10 = 0.0;
  while (PMSerial.available())
  {
    //Serial.print("PMSerial available");
    PMRev_buf[PMCounter] = (unsigned char)PMSerial.read();
    Serial.print(PMRev_buf[PMCounter], HEX);
    if (PMCounter == 0 && PMRev_buf[0] != 0XAA)
      continue;
    if (PMCounter == 1 && PMRev_buf[1] != 0XC0)
    {
      PMCounter = 0;
      continue;
    }
    PMCounter++;
    if (PMCounter == 10)
    {
      PMCounter = 0;
      PMSign = 1;
      break;
    }
  }
  if (PMSign) 
  {
    PMSign = 0;
    if (PMRev_buf[0] == 0XAA && PMRev_buf[1] == 0XC0)
    {
      for (PM_i = 0 + 2; PM_i < 6 + 2; PM_i++)
      {
        // ---------------------------------------------
        // AA -- 报文头
        // C0 -- 指令号，表示是由PM2.5传感器输出的信号
        // 71 -- PM2.5 低字节
        // 01 -- PM2.5 高字节
        // CA -- PM10 低字节
        // 01 -- PM10 高字节
        // B9 -- 传感器ID
        // 93 -- 传感器ID
        // 89 -- 校验和 (71+01+CA+01+B9+93) & 0XFF
        // AB -- 报文尾
        // ---------------------------------------------
        PMChecksum += PMRev_buf[PM_i];
      }
      if (PMChecksum == PMRev_buf[PM_i])
      {
        // Checksum Right
        PM2_5 = ((PMRev_buf[3] * 256) + PMRev_buf[2]) / 10;
        PM10 = ((PMRev_buf[5] * 256) + PMRev_buf[4]) / 10;
        Serial.print("\nPM2.5: ");
        Serial.println(PM2_5);
        Serial.print("PM10: ");
        Serial.println(PM10);
        PM2_5_TEMP = PM2_5;
        PM10_TEMP = PM10;
      }
      delay(500);
    }
  }
  if (PM2_5 == 0.00)
  {
    PM2_5 = PM2_5_TEMP;
  }
  if (PM10 == 0.00)
  {
    PM10 = PM10_TEMP;
  }

  // GY_MCU680=================================================
  float Temperature = 0.00, Humidity = 0.00;
  unsigned char GYMCU680_i = 0;
  unsigned char GYMCU680Checksum = 0;
  uint32_t Gas;
  uint32_t Pressure;
  uint32_t IAQ = 0;
  int16_t Altitude;
  uint8_t IAQ_accuracy;
  while (GYMCU680Serial.available())
  {
    GYMCU680Rev_buf[GYMCU680Counter] = (unsigned char)GYMCU680Serial.read();
    Serial.print(GYMCU680Rev_buf[GYMCU680Counter], HEX);
    if (GYMCU680Counter == 0 && GYMCU680Rev_buf[0] != 0X5A)
      continue;
    if (GYMCU680Counter == 1 && GYMCU680Rev_buf[1] != 0X5A)
    {
      GYMCU680Counter = 0;
      continue;
    }
    GYMCU680Counter++;
    if (GYMCU680Counter == 20)
    {
      GYMCU680Counter = 0;
      GYMCU680Sign = 1;
      break;
    }
  }
  Serial.println();
  if (GYMCU680Sign)
  {
    GYMCU680Sign = 0;
    if (GYMCU680Rev_buf[0] == 0X5A && GYMCU680Rev_buf[1] == 0X5A)
    {
      for (GYMCU680_i = 0; GYMCU680_i < 19; GYMCU680_i++)
      {
        GYMCU680Checksum += GYMCU680Rev_buf[GYMCU680_i];
      }
      if (GYMCU680Checksum == GYMCU680Rev_buf[GYMCU680_i])
      {
        temp2 = (GYMCU680Rev_buf[4] << 8 | GYMCU680Rev_buf[5]);
        Temperature = (float)temp2 / 100;
        Temperature_TEMP = Temperature;
        temp1 = (GYMCU680Rev_buf[6] << 8 | GYMCU680Rev_buf[7]);
        Humidity = (float)temp1 / 100;
        Humidity_TEMP = Humidity;
        Pressure = ((uint32_t)GYMCU680Rev_buf[8] << 16) | ((uint16_t)GYMCU680Rev_buf[9] << 8) | GYMCU680Rev_buf[10];
        IAQ_accuracy = (GYMCU680Rev_buf[11] & 0XF0) >> 4;
        IAQ = ((GYMCU680Rev_buf[11] & 0X0F) << 8) | GYMCU680Rev_buf[12];
        IAQ_TEMP = IAQ;
        Gas = ((uint32_t)GYMCU680Rev_buf[13] << 24) | ((uint32_t)GYMCU680Rev_buf[14] << 16) | ((uint16_t)GYMCU680Rev_buf[15] << 8) | GYMCU680Rev_buf[16];
        Altitude = (GYMCU680Rev_buf[17] << 8) | GYMCU680Rev_buf[18];
        Serial.print("Temperature:");
        Serial.println(Temperature);
        Serial.print("Humidity:");
        Serial.println(Humidity);
        Serial.print("Pressure:");
        Serial.println(Pressure);
        Serial.print("IAQ:");
        Serial.println(IAQ);
        Serial.print("Gas:");
        Serial.println(Gas);
        Serial.print("Altitude:");
        Serial.println(Altitude);
        Serial.print("IAQ_accuracy:");
        Serial.println(IAQ_accuracy);
      }
      delay(500);
    }
  }
  if (Temperature == 0.00)
  {
    Temperature = Temperature_TEMP;
  }
  if (Humidity == 0.00)
  {
    Humidity = Humidity_TEMP;
  }
  if (IAQ == 0)
  {
    IAQ = IAQ_TEMP;
  }

  // MQ-4================================================
  CH4 = analogRead(AOUTpin);
  limit = digitalRead(DOUTpin);
  Serial.print("Methane CH4: ");
  Serial.println(CH4);
  Serial.print("Limit: ");
  Serial.println(limit);

  // SGP30=====================================================
  sgp.setHumidity(getAbsoluteHumidity(Temperature, Humidity));
  if (!sgp.IAQmeasure())
  {
    Serial.println("Measurement failed");
  }
  uint16_t TVOC = sgp.TVOC;
  uint16_t eCO2 = sgp.eCO2;
  Serial.print("eCO2: ");
  Serial.print(eCO2);
  Serial.println(" ppm");
  Serial.print("TVOC: ");
  Serial.print(TVOC);
  Serial.println(" ppb");

  S_Temperature = String("Temperature: ") + String(Temperature) + String(" C");
  S_Humidity = String("Humidity: ") + String(Humidity) + String("%");
  S_CH4 = String("CH4: ") + String(CH4) + String(" ppm");
  S_IAQ = String("IAQ: ") + String(IAQ);
  S_PM2_5 = String("PM2.5: ") + String(PM2_5) + String(" ug/m3");
  S_PM10 = String("PM10: ") + String(PM10) + String(" ug/m3");
  S_eCO2 = String("eCO2: ") + String(eCO2) + String(" ppm");
  S_TVOC = String("TVOC: ") + String(TVOC) + String(" ppm");

  // DATA format: INFO\t{MAC}\t{Temperature}\t{Humidity}\t{IAQ}\t{CH4}\t{PM2_5}\t{PM10}\t{eCO2}\t{TVOC}@END
  String DATA = "INFO\t" + String(WiFi.macAddress()) + "\t" + String(Temperature) + "\t" + String(Humidity) + "\t" + String(IAQ) +
                "\t" + String(CH4) + "\t" + String(PM2_5) + "\t" + String(PM10) + "\t" +
                String(eCO2) + "\t" + String(TVOC) + "@END";
  return DATA;
}

void sendData(String DATA)
{
  // String DATA = getData();
  // DATA format: INFO\t{MAC}\t{Temperature}\t{Humidity}\t{IAQ}\t{CH4}\t{PM2_5}\t{PM10}\t{eCO2}\t{TVOC}@END
  char senddata[DATA.length() + 1];
  DATA.toCharArray(senddata, DATA.length() + 1);
  dataUdp.beginPacket(serverip, sendport);
  dataUdp.printf("%s", senddata);
  dataUdp.endPacket();
}

void connectAPP()
{
  Serial.print("Connecting to ");
  Serial.println(ssid);

  OLED_Clear();
  OLED_ShowString(0, 26, "Connecting To Phone", 12);
  OLED_Refresh();

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  connectedAPP = true;
}

void connectRoute()
{
  while (1)
  {
    // 从手机端接收家庭路由器的ssid和password
    // format: \t{ssid}\t\r{password}\r\n
    int packetSize = Udp.parsePacket();
    if (packetSize)
    {
      char buf[packetSize];
      Udp.read(buf, packetSize);

      Serial.println();
      Serial.print("Recevied: ");
      Serial.println(buf);

      Serial.print("From IP: ");
      Serial.println(Udp.remoteIP());
      Serial.print("From Port: ");
      Serial.println(Udp.remotePort());

      // 原文回复
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.print("Recevied: ");
      Udp.write((const uint8_t *)buf, packetSize);
      Udp.endPacket();

      int i = 0;
      int id_ptr = 0;
      int pw_ptr = 0;
      while (i < packetSize)
      {
        if (buf[i] == '\t')
        {
          i = i + 1;
          id_ptr = i;
          while (buf[i] != '\t')
          {
            i = i + 1;
          }
          strncpy(new_id, buf + id_ptr, i - id_ptr);
          i = i + 1;
        }
        else if (buf[i] == '\r')
        {
          i = i + 1;
          pw_ptr = i;
          while (buf[i] != '\r')
          {
            i = i + 1;
          }
          strncpy(new_pw, buf + pw_ptr, i - pw_ptr);
          i = i + 1;
        }
        else if (buf[i] == '\n')
        {
          break;
        }
      }

      // 得到路由器的ssid和密码
      new_ssid = new_id;
      new_password = new_pw;
      Serial.print("New ssid: ");
      Serial.println(new_ssid);
      Serial.print("New password: ");
      Serial.println(new_password);

      // 连接到路由器
      WiFi.mode(WIFI_STA);
      WiFi.disconnect();
      delay(100);

      // 显示正在连接信息
      OLED_Clear();
      OLED_ShowString(0, 26, "Connecting To Route", 12);
      OLED_Refresh();

      Serial.println("Now get the new ssid and password, connecting to route!");
      WiFi.begin(new_ssid, new_password);
      while (WiFi.status() != WL_CONNECTED)
      {
        delay(500);
      }

      Serial.println("WiFi connected!");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectedRoute = true;
      return;
    }
  }
}

//反显函数
void OLED_ColorTurn(uint8_t i)
{
  if (!i)
    OLED_WR_Byte(0xA6, OLED_CMD); //正常显示
  else
    OLED_WR_Byte(0xA7, OLED_CMD); //反色显示
}

//屏幕旋转180度
void OLED_DisplayTurn(uint8_t i)
{
  if (i == 0)
  {
    OLED_WR_Byte(0xC8, OLED_CMD); //正常显示
    OLED_WR_Byte(0xA1, OLED_CMD);
  }
  else
  {
    OLED_WR_Byte(0xC0, OLED_CMD); //反转显示
    OLED_WR_Byte(0xA0, OLED_CMD);
  }
}
//起始信号
void I2C_Start(void)
{
  OLED_SDIN_Set();
  OLED_SCLK_Set();
  OLED_SDIN_Clr();
  OLED_SCLK_Clr();
}

//结束信号
void I2C_Stop(void)
{
  OLED_SCLK_Set();
  OLED_SDIN_Clr();
  OLED_SDIN_Set();
}

//等待信号响应
void I2C_WaitAck(void) //测数据信号的电平
{
  OLED_SCLK_Set();
  OLED_SCLK_Clr();
}

//写入一个字节
void Send_Byte(uint8_t dat)
{
  uint8_t i;
  for (i = 0; i < 8; i++)
  {
    OLED_SCLK_Clr(); //将时钟信号设置为低电平
    if (dat & 0x80)  //将dat的8位从最高位依次写入
    {
      OLED_SDIN_Set();
    }
    else
    {
      OLED_SDIN_Clr();
    }
    OLED_SCLK_Set(); //将时钟信号设置为高电平
    OLED_SCLK_Clr(); //将时钟信号设置为低电平
    dat <<= 1;
  }
}

//发送一个字节
//向SSD1306写入一个字节。
//mode:数据/命令标志 0,表示命令;1,表示数据;
void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
  I2C_Start();
  Send_Byte(0x78);
  I2C_WaitAck();
  if (mode)
  {
    Send_Byte(0x40);
  }
  else
  {
    Send_Byte(0x00);
  }
  I2C_WaitAck();
  Send_Byte(dat);
  I2C_WaitAck();
  I2C_Stop();
}

//更新显存到OLED
void OLED_Refresh(void)
{
  uint8_t i, n;
  for (i = 0; i < 8; i++)
  {
    OLED_WR_Byte(0xb0 + i, OLED_CMD); //设置行起始地址
    OLED_WR_Byte(0x02, OLED_CMD);     //设置低列起始地址
    OLED_WR_Byte(0x10, OLED_CMD);     //设置高列起始地址
    for (n = 0; n < 128; n++)
      OLED_WR_Byte(OLED_GRAM[n][i], OLED_DATA);
  }
}
//清屏函数
void OLED_Clear(void)
{
  uint8_t i, n;
  for (i = 0; i < 8; i++)
  {
    for (n = 0; n < 128; n++)
    {
      OLED_GRAM[n][i] = 0; //清除所有数据
    }
  }
  OLED_Refresh(); //更新显示
}

//画点
//x:0~127
//y:0~63
void OLED_DrawPoint(uint8_t x, uint8_t y)
{
  uint8_t i, m, n;
  i = y / 8;
  m = y % 8;
  n = 1 << m;
  OLED_GRAM[x][i] |= n;
}

//清除一个点
//x:0~127
//y:0~63
void OLED_ClearPoint(uint8_t x, uint8_t y)
{
  uint8_t i, m, n;
  i = y / 8;
  m = y % 8;
  n = 1 << m;
  OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
  OLED_GRAM[x][i] |= n;
  OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
}

//画线
//x:0~128
//y:0~64
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
  uint8_t i, k, k1, k2, y0;
  if (x1 == x2) //画竖线
  {
    for (i = 0; i < (y2 - y1); i++)
    {
      OLED_DrawPoint(x1, y1 + i);
    }
  }
  else if (y1 == y2) //画横线
  {
    for (i = 0; i < (x2 - x1); i++)
    {
      OLED_DrawPoint(x1 + i, y1);
    }
  }
  else //画斜线
  {
    k1 = y2 - y1;
    k2 = x2 - x1;
    k = k1 * 10 / k2;
    for (i = 0; i < (x2 - x1); i++)
    {
      OLED_DrawPoint(x1 + i, y1 + i * k / 10);
    }
  }
}
//x,y:圆心坐标
//r:圆的半径
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r)
{
  int a, b, num;
  a = 0;
  b = r;
  while (2 * b * b >= r * r)
  {
    OLED_DrawPoint(x + a, y - b);
    OLED_DrawPoint(x - a, y - b);
    OLED_DrawPoint(x - a, y + b);
    OLED_DrawPoint(x + a, y + b);

    OLED_DrawPoint(x + b, y + a);
    OLED_DrawPoint(x + b, y - a);
    OLED_DrawPoint(x - b, y - a);
    OLED_DrawPoint(x - b, y + a);

    a++;
    num = (a * a + b * b) - r * r; //计算画的点离圆心的距离
    if (num > 0)
    {
      b--;
      a--;
    }
  }
}

//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//size:选择字体 12/16/24
//取模方式 逐列式
void OLED_ShowChar(uint8_t x, uint8_t y, const char chr, uint8_t size1)
{
  uint8_t i, m, temp, size2, chr1;
  uint8_t y0 = y;
  size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2); //得到字体一个字符对应点阵集所占的字节数
  chr1 = chr - ' ';                                          //计算偏移后的值
  for (i = 0; i < size2; i++)
  {
    if (size1 == 12)
    {
      temp = pgm_read_byte(&asc2_1206[chr1][i]);
    } //调用1206字体
    else if (size1 == 16)
    {
      temp = pgm_read_byte(&asc2_1608[chr1][i]);
    } //调用1608字体
    else if (size1 == 24)
    {
      temp = pgm_read_byte(&asc2_2412[chr1][i]);
    } //调用2412字体
    else
      return;
    for (m = 0; m < 8; m++) //写入数据
    {
      if (temp & 0x80)
        OLED_DrawPoint(x, y);
      else
        OLED_ClearPoint(x, y);
      temp <<= 1;
      y++;
      if ((y - y0) == size1)
      {
        y = y0;
        x++;
        break;
      }
    }
  }
}

//显示字符串
//x,y:起点坐标
//size1:字体大小
//*chr:字符串起始地址
void OLED_ShowString(uint8_t x, uint8_t y, const char *chr, uint8_t size1)
{
  while ((*chr >= ' ') && (*chr <= '~')) //判断是不是非法字符!
  {
    OLED_ShowChar(x, y, *chr, size1);
    x += size1 / 2;
    if (x > 128 - size1 / 2) //换行
    {
      x = 0;
      y += size1;
    }
    chr++;
  }
}

//m^n
u32 OLED_Pow(uint8_t m, uint8_t n)
{
  u32 result = 1;
  while (n--)
  {
    result *= m;
  }
  return result;
}

////显示2个数字
////x,y :起点坐标
////len :数字的位数
////size:字体大小
void OLED_ShowNum(uint8_t x, uint8_t y, int num, uint8_t len, uint8_t size1)
{
  uint8_t t, temp;
  for (t = 0; t < len; t++)
  {
    temp = (num / OLED_Pow(10, len - t - 1)) % 10;
    if (temp == 0)
    {
      OLED_ShowChar(x + (size1 / 2) * t, y, '0', size1);
    }
    else
    {
      OLED_ShowChar(x + (size1 / 2) * t, y, temp + '0', size1);
    }
  }
}

//显示汉字
//x,y:起点坐标
//num:汉字对应的序号
//取模方式 列行式
void OLED_ShowChinese(uint8_t x, uint8_t y, const uint8_t num, uint8_t size1)
{
  uint8_t i, m, n = 0, temp, chr1;
  uint8_t x0 = x, y0 = y;
  uint8_t size3 = size1 / 8;
  while (size3--)
  {
    chr1 = num * size1 / 8 + n;
    n++;
    for (i = 0; i < size1; i++)
    {
      if (size1 == 16)
      {
        temp = pgm_read_byte(&Hzk1[chr1][i]);
      } //调用16*16字体
      else if (size1 == 24)
      {
        temp = pgm_read_byte(&Hzk2[chr1][i]);
      } //调用24*24字体
      else if (size1 == 32)
      {
        temp = pgm_read_byte(&Hzk3[chr1][i]);
      } //调用32*32字体
      else if (size1 == 64)
      {
        temp = pgm_read_byte(&Hzk4[chr1][i]);
      } //调用64*64字体
      else
        return;

      for (m = 0; m < 8; m++)
      {
        if (temp & 0x01)
          OLED_DrawPoint(x, y);
        else
          OLED_ClearPoint(x, y);
        temp >>= 1;
        y++;
      }
      x++;
      if ((x - x0) == size1)
      {
        x = x0;
        y0 = y0 + 8;
      }
      y = y0;
    }
  }
}

//配置写入数据的起始位置
void OLED_WR_BP(uint8_t x, uint8_t y)
{
  x += 2;
  OLED_WR_Byte(0xb0 + y, OLED_CMD); //设置行起始地址
  OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
  OLED_WR_Byte((x & 0x0f), OLED_CMD);
}

//x0,y0：起点坐标
//x1,y1：终点坐标
//BMP[]：要写入的图片数组
void OLED_ShowPicture(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t BMP[])
{
  int j = 0;
  uint8_t t;
  uint8_t x, y;
  for (y = y0; y < y1; y++)
  {
    OLED_WR_BP(x0, y);
    for (x = x0; x < x1; x++)
    {
      t = pgm_read_byte(&BMP[j++]);
      OLED_WR_Byte(t, OLED_DATA);
    }
  }
}

//OLED的初始化
void OLED_Init(void)
{
  pinMode(scl, OUTPUT); //设置数字8
  pinMode(sda, OUTPUT); //设置数字9

  delay(100);

  OLED_WR_Byte(0xAE, OLED_CMD); //--turn off oled panel
  OLED_WR_Byte(0x00, OLED_CMD); //---set low column address
  OLED_WR_Byte(0x10, OLED_CMD); //---set high column address
  OLED_WR_Byte(0x40, OLED_CMD); //--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
  OLED_WR_Byte(0x81, OLED_CMD); //--set contrast control register
  OLED_WR_Byte(0xCF, OLED_CMD); // Set SEG Output Current Brightness
  OLED_WR_Byte(0xA1, OLED_CMD); //--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
  OLED_WR_Byte(0xC8, OLED_CMD); //Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
  OLED_WR_Byte(0xA6, OLED_CMD); //--set normal display
  OLED_WR_Byte(0xA8, OLED_CMD); //--set multiplex ratio(1 to 64)
  OLED_WR_Byte(0x3f, OLED_CMD); //--1/64 duty
  OLED_WR_Byte(0xD3, OLED_CMD); //-set display offset Shift Mapping RAM Counter (0x00~0x3F)
  OLED_WR_Byte(0x00, OLED_CMD); //-not offset
  OLED_WR_Byte(0xd5, OLED_CMD); //--set display clock divide ratio/oscillator frequency
  OLED_WR_Byte(0x80, OLED_CMD); //--set divide ratio, Set Clock as 100 Frames/Sec
  OLED_WR_Byte(0xD9, OLED_CMD); //--set pre-charge period
  OLED_WR_Byte(0xF1, OLED_CMD); //Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
  OLED_WR_Byte(0xDA, OLED_CMD); //--set com pins hardware configuration
  OLED_WR_Byte(0x12, OLED_CMD);
  OLED_WR_Byte(0xDB, OLED_CMD); //--set vcomh
  OLED_WR_Byte(0x40, OLED_CMD); //Set VCOM Deselect Level
  OLED_WR_Byte(0x20, OLED_CMD); //-Set Page Addressing Mode (0x00/0x01/0x02)
  OLED_WR_Byte(0x02, OLED_CMD); //
  OLED_WR_Byte(0x8D, OLED_CMD); //--set Charge Pump enable/disable
  OLED_WR_Byte(0x14, OLED_CMD); //--set(0x10) disable
  OLED_WR_Byte(0xA4, OLED_CMD); // Disable Entire Display On (0xa4/0xa5)
  OLED_WR_Byte(0xA6, OLED_CMD); // Disable Inverse Display On (0xa6/a7)
  OLED_WR_Byte(0xAF, OLED_CMD);
  OLED_Clear();
}
