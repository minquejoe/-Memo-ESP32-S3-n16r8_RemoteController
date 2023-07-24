//<App !Start!>
// FILE: [main.cpp]
// Created by GUIslice Builder version: [0.17.b24]
//
// GUIslice Builder Generated File
//
// For the latest guides, updates and support view:
// https://github.com/ImpulseAdventure/GUIslice
//
//<App !End!>

// ------------------------------------------------
// Headers to include
// ------------------------------------------------
#include "test2_GSLC.h"

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>
#include "DFRobot_BMM150.h"
#include <Bounce2.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <String.h>

// ------------------------------------------------
// Program Globals
// ------------------------------------------------

// Save some element references for direct access
//<Save_References !Start!>
gslc_tsElemRef *m_pElem_power = NULL;
gslc_tsElemRef *m_pElem_north = NULL;
gslc_tsElemRef *m_pElem_T = NULL;
gslc_tsElemRef *m_pElem_RH = NULL;
gslc_tsElemRef *m_pElem_P = NULL;
gslc_tsElemRef *m_pElem_Alti = NULL;
gslc_tsElemRef *m_pElem_mag_x = NULL;
gslc_tsElemRef *m_pElem_mag_y = NULL;
gslc_tsElemRef *m_pElem_mag_z = NULL;
gslc_tsElemRef *m_pElem_conn = NULL;
//<Save_References !End!>

// Define debug message function
static int16_t DebugOut(char ch)
{
  if (ch == (char)'\n')
    Serial.println("");
  else
    Serial.write(ch);
  return 0;
}

// ------------------------------------------------
// Callback Methods
// ------------------------------------------------
//<Button Callback !Start!>
//<Button Callback !End!>
//<Checkbox Callback !Start!>
//<Checkbox Callback !End!>
//<Keypad Callback !Start!>
//<Keypad Callback !End!>
//<Spinner Callback !Start!>
//<Spinner Callback !End!>
//<Listbox Callback !Start!>
//<Listbox Callback !End!>
//<Draw Callback !Start!>
//<Draw Callback !End!>
//<Slider Callback !Start!>
//<Slider Callback !End!>
//<Tick Callback !Start!>
//<Tick Callback !End!>

// 没焊好，无效
// Adafruit_AHTX0 aht;
// DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_4);

// bmp 数据块
typedef struct
{
  TickType_t measure_interval;
  volatile float T;
  volatile float P;
  volatile float Alti;
} BMP_SENSORS;

BMP_SENSORS bmp_sensor;

// 屏幕输出字符缓存区
char acTxt[7];

// 数据刷新是否忙指标
SemaphoreHandle_t xMutex_flag_reshScreen = NULL;
SemaphoreHandle_t xMutex_flag_anaUpdate = NULL;

void bmp_meansure(void *pt)
{
  // 以绝对频率运行程序
  const TickType_t xFreq = bmp_sensor.measure_interval;
  TickType_t xLastwakeTime = xTaskGetTickCount();

  // 压力传感器初始化
  Adafruit_BMP280 bmp;

  if (bmp.begin())
    Serial.println("bmp init success!");
  else
  {
    Serial.println("bmp init fail! measure task delete!");
    vTaskDelete(NULL);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  for (;;)
  {
    // must call this to wake sensor up and get new measurement data
    // it blocks until measurement is complete
    if (bmp.takeForcedMeasurement())
    {
      if (xSemaphoreTake(xMutex_flag_reshScreen, xFreq) == pdPASS)
      {
        bmp_sensor.T = bmp.readTemperature();
        bmp_sensor.P = bmp.readPressure() / 1e3;
        bmp_sensor.Alti = bmp.readAltitude(1013.25);
        xSemaphoreGive(xMutex_flag_reshScreen);
      }
    }

    // vTaskDelay(xFreq);
    // 下面这么做的好处是，当测量消耗的时间不确定时，可以使等待时间跟随变化
    // 绝对频率运行
    vTaskDelayUntil(&xLastwakeTime, xFreq);
  }
}

// 摇杆数据块
typedef struct
{
  byte pin;
  byte filter_order;
  // byte filter_threshold;
  volatile u32_t filtered_val;
  volatile u8_t filtered_val_norm100;
  byte update_interval;
  u16_t ana_max;
  u16_t ana_min;
} ANA_SET;
ANA_SET stick_x, stick_y;

void ana_filter(void *pt)
{
  ANA_SET *ana_set = (ANA_SET *)pt;
  // 给摇杆的模拟量的限幅平均滤波
  // 限幅效果不好，不限幅了
  byte pin = ana_set->pin;
  byte filter_order = ana_set->filter_order;
  // byte filter_threshold = ana_set->filter_threshold;
  byte update_interval = ana_set->update_interval;
  volatile u32_t *filtered_val = &ana_set->filtered_val;
  volatile u8_t *filtered_val_norm100 = &ana_set->filtered_val_norm100;
  // u16_t ana_max = ana_set->ana_max;
  u16_t ana_min = ana_set->ana_min;

  pinMode(pin, INPUT);

  // u32_t ana_buffer[filter_order] = {analogRead(pin)};
  u32_t ana_buffer[filter_order];
  byte buffer_idx = 0;
  u16_t ana_max;

  // 自动设置 ana_max
  delay(500); // 等开机完成
  for (u8_t i; i < filter_order; i++)
  {
    ana_buffer[i] = analogRead(pin);
    delay(1);
  }
  for (u32_t val : ana_buffer)
    *filtered_val = *filtered_val + val;
  *filtered_val = round(*filtered_val / filter_order);
  ana_max = 2 * *filtered_val - ana_min;

  for (;;)
  {
    ana_buffer[buffer_idx] = analogRead(pin);
    // if (abs((int)(ana_buffer[buffer_idx] - ana_buffer[(buffer_idx + filter_order - 1) % filter_order])) > filter_threshold)
    // {
    if (xSemaphoreTake(xMutex_flag_anaUpdate, update_interval) == pdPASS)
    {
      for (u32_t val : ana_buffer)
        *filtered_val = *filtered_val + val;
      *filtered_val = round(*filtered_val / filter_order);
      *filtered_val_norm100 = round(100 * (*filtered_val - ana_min) / (ana_max - ana_min));
      // 直接限幅
      if (*filtered_val_norm100 > 110)
      {
        // ana_min 没设置好的变量溢出
        *filtered_val_norm100 = 0;
      }
      else if (*filtered_val_norm100 > 100)
      {
        // ana_max 没设置好的大变量值
        *filtered_val_norm100 = 100;
      }

      xSemaphoreGive(xMutex_flag_anaUpdate);
    }
    buffer_idx = (buffer_idx + 1) % filter_order;
    // }

    vTaskDelay(update_interval);
  }
}

// 按键
// 上 下 左 右 （右侧由上至下）
const byte keyNums = 8;
byte keyPins[keyNums] = {47, 9, 46, 21, 14, 39, 0, 45};
Bounce bounce[keyNums];
u8_t power = 50; // 能量输出

// 蓝牙
BLECharacteristic *pCharacteristic;          // 创建一个BLE特性pCharacteristic
bool deviceConnected = false;                // 连接否标志位
uint8_t txValue = 0;                         // TX的值
long lastMsg = 0;                            // 存放时间的变量
String rxload = "ESP32-S3-RemoteController"; // RX的预置值
uint8_t ble_interval = 100;

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// 服务器回调
// 继承自 BLEServerCallbacks 进行了复写
char flag_ble_conn[4];
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    pServer->getAdvertising()->start(); // 服务器开始广播
  }
};

// 特性回调
class BLE_Wirte_Callbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    // 打印收到的值
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0)
    {
      rxload = "";
      for (int i = 0; i < rxValue.length(); i++)
      {
        rxload += (char)rxValue[i];
        Serial.print(rxValue[i]);
      }
      Serial.println("");
    }
  }
};

// 蓝牙初始化函数
void setupBLE(String BLEName)
{
  const char *ble_name = BLEName.c_str(); // 将传入的BLE的名字转换为指针
  BLEDevice::init(ble_name);              // 初始化一个蓝牙设备

  BLEServer *pServer = BLEDevice::createServer(); // 创建一个蓝牙服务器
  pServer->setCallbacks(new MyServerCallbacks()); // 服务器回调函数设置为MyServerCallbacks

  BLEService *pService = pServer->createService(SERVICE_UUID); // 创建一个BLE服务

  // 创建一个(读)特征值 类型是通知
  // 全局特征
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  // 为特征添加一个描述
  pCharacteristic->addDescriptor(new BLE2902());

  // 创建一个(写)特征 类型是写入
  // 函数内创建，不全局
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  // 为特征添加一个回调
  pCharacteristic->setCallbacks(new BLE_Wirte_Callbacks());

  pService->start();                  // 开启服务
  pServer->getAdvertising()->start(); // 服务器开始广播
  Serial.println("Waiting a client connection to notify...");
}

void ble_server(void *pt)
{
  setupBLE("ESP32-S3-BLE"); // 设置蓝牙名称

  // 以绝对频率运行程序
  const TickType_t xFreq = ble_interval; // ms
  TickType_t xLastwakeTime = xTaskGetTickCount();

  for (;;)
  {
    // notify debug 通知测试
    if (deviceConnected && rxload.length() > 0)
    {
      String str = rxload;
      if (str == "notify debug\r\n")
      {
        const char *newValue = str.c_str();
        pCharacteristic->setValue(newValue);
        pCharacteristic->notify();
      }
    }

    char ctrl_buf[12];
    snprintf(ctrl_buf, 13, "%d %d %d\r\n", stick_x.filtered_val_norm100, stick_y.filtered_val_norm100, power);
    pCharacteristic->setValue(ctrl_buf);
    pCharacteristic->notify();

    // 每隔一段时间运行一下服务
    vTaskDelayUntil(&xLastwakeTime, xFreq);
  }
}

void setup()
{
  // ------------------------------------------------
  // Initialize
  // ------------------------------------------------
  Serial.begin(9600);
  // Wait for USB Serial
  // delay(1000);  // NOTE: Some devices require a delay after Serial.begin() before serial port can be used

  gslc_InitDebug(&DebugOut);

  Wire.begin(5, 4); // join i2c bus (address optional for master)

  // // 温湿度传感器 Adfruit 库
  // if (aht.begin())
  // {
  //   Serial.println("Found AHT20");
  // }
  // else
  // {
  //   Serial.println("Didn't find AHT20");
  // }

  // // 陀螺仪
  // while (bmm150.begin())
  // {
  //   Serial.println("bmm150 init failed, Please try again!");
  //   delay(1000);
  // }
  // Serial.println("bmm150 init success!");
  // bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
  // bmm150.setPresetMode(BMM150_PRESETMODE_REGULAR);
  // bmm150.setRate(BMM150_DATA_RATE_10HZ);
  // bmm150.setMeasurementXYZ();

  // ------------------------------------------------
  // Create graphic elements
  // ------------------------------------------------
  InitGUIslice_gen();

  // ------------------------------------------------
  // FreeRTOS
  // ------------------------------------------------
  // bmp
  xMutex_flag_reshScreen = xSemaphoreCreateMutex();
  bmp_sensor.measure_interval = 1000; // ms
  if (xTaskCreatePinnedToCore(bmp_meansure, "DO measure", 1024 * 8, NULL, 1, NULL, 1) == pdPASS)
    Serial.println("bmp measure task created!");

  // 摇杆
  xMutex_flag_anaUpdate = xSemaphoreCreateMutex();

  // 平均阶数，影响 ana_max
  stick_x.filter_order = 128;
  // 阈值太大，导致大幅的变动影响更显著，小幅变动平均效果变差
  // stick_x.filter_threshold = 0;
  stick_x.pin = 17;
  stick_x.update_interval = 1; // ms
  // ADC 默认12位分辨率 0 - 4095
  // 直接初始化时候计算了
  // stick_x.ana_max = 4095 + 1245;
  stick_x.ana_min = 0 + 100;
  if (xTaskCreatePinnedToCore(ana_filter, "DO X", 1024 * 8, (void *)&stick_x, 1, NULL, 1) == pdPASS)
    Serial.println("ANA X task created!");

  stick_y = stick_x;
  stick_y.pin = 3;
  if (xTaskCreatePinnedToCore(ana_filter, "DO Y", 1024 * 8, (void *)&stick_y, 1, NULL, 1) == pdPASS)
    Serial.println("ANA Y task created!");

  // 十字按键
  for (int i = 0; i < keyNums; i++)
  {
    bounce[i].attach(keyPins[i], INPUT_PULLUP);
    bounce[i].interval(5);
  }

  // 蓝牙
  if (xTaskCreatePinnedToCore(ble_server, "DO BLE", 1024 * 8, NULL, 1, NULL, 0) == pdPASS)
    Serial.println("BLE task created!");
}

// -----------------------------------
// Main event loop
// -----------------------------------
typedef struct
{
  u8_t x;
  u8_t y;

} POINT_POS;
POINT_POS pos_old = {50, 50};
POINT_POS pos_new;

void loop()
{

  // 没焊好，未启用
  snprintf(acTxt, 6, (char *)"NULL");
  gslc_ElemSetTxtStr(&m_gui, m_pElem_RH, acTxt);

  snprintf(acTxt, 4, (char *)"NaN");
  gslc_ElemSetTxtStr(&m_gui, m_pElem_mag_x, acTxt);

  snprintf(acTxt, 4, (char *)"NaN");
  gslc_ElemSetTxtStr(&m_gui, m_pElem_mag_y, acTxt);

  snprintf(acTxt, 4, (char *)"NaN");
  gslc_ElemSetTxtStr(&m_gui, m_pElem_mag_z, acTxt);

  // bmp 传感器不在写变量的时候更新
  if (xSemaphoreTake(xMutex_flag_reshScreen, 0) == pdPASS)
  {
    snprintf(acTxt, 5, "%f", bmp_sensor.T);
    gslc_ElemSetTxtStr(&m_gui, m_pElem_T, acTxt);

    snprintf(acTxt, 7, "%f", bmp_sensor.P);
    gslc_ElemSetTxtStr(&m_gui, m_pElem_P, acTxt);

    snprintf(acTxt, 7, "%f", bmp_sensor.Alti);
    gslc_ElemSetTxtStr(&m_gui, m_pElem_Alti, acTxt);

    xSemaphoreGive(xMutex_flag_reshScreen);
  }

  // 按键状态更新
  for (int i = 0; i < keyNums; i++)
  {
    bounce[i].update();
  }

  // 十字 上
  if (bounce[0].fell())
  {
    if (power <= 90)
      power += 10;
    else
      power = 100;
  }

  // 十字 下
  if (bounce[1].fell())
  {
    if (power >= 10)
      power -= 10;
    else
      power = 0;
  }

  // 十字 右
  if (bounce[3].fell())
  {
    power = 50;
  }

  // 右侧 1
  if (bounce[4].fell())
  {
    power = 100;
  }

  // 右侧 4
  if (bounce[7].fell())
  {
    power = 0;
  }

  // ------------------------------------------------
  // Update GUI Elements
  // ------------------------------------------------

  // TODO - Add update code for any text, gauges, or sliders

  gslc_ElemXRampSetVal(&m_gui, m_pElem_power, power);
  // gslc_ElemXRadialSetVal(&m_gui, m_pElem_north, power_count % 360 + 1);

  if (xSemaphoreTake(xMutex_flag_anaUpdate, 0) == pdPASS)
  {
    pos_new.x = round(120 / 100 * stick_x.filtered_val_norm100);
    pos_new.y = round(120 / 100 * stick_y.filtered_val_norm100);
    xSemaphoreGive(xMutex_flag_anaUpdate);
  }

  if ((pos_new.x != pos_old.x) || (pos_new.y != pos_old.y))
  {
    // 动点底盘
    gslc_DrawFillRoundRect(&m_gui, (gslc_tsRect){(int16_t)(30), (int16_t)(70), 140, 140}, 10, GSLC_COL_BLUE);

    // 动点
    gslc_DrawFillCircle(&m_gui,
                        50 + pos_new.x,
                        // 屏幕翻转了，使用 -
                        190 - pos_new.y,
                        5,
                        GSLC_COL_RED);

    pos_old.x = pos_new.x;
    pos_old.y = pos_new.y;
  }

  // 动点底盘三角 左 右 上 下
  gslc_DrawFillTriangle(&m_gui, 20, 135, 29, 140, 20, 145, GSLC_COL_WHITE);
  gslc_DrawFillTriangle(&m_gui, 178, 135, 171, 140, 178, 145, GSLC_COL_WHITE);
  gslc_DrawFillTriangle(&m_gui, 95, 61, 100, 69, 105, 61, GSLC_COL_WHITE);
  gslc_DrawFillTriangle(&m_gui, 95, 219, 100, 211, 105, 219, GSLC_COL_WHITE);

  // 蓝牙连接指示
  if (deviceConnected)
  {
    snprintf(acTxt, 4, (char *)"ON!");
    gslc_ElemSetTxtCol(&m_gui, m_pElem_conn, GSLC_COL_WHITE);
  }
  else
  {
    snprintf(acTxt, 4, (char *)"OFF");
    gslc_ElemSetTxtCol(&m_gui, m_pElem_conn, GSLC_COL_YELLOW);
  }
  gslc_ElemSetTxtStr(&m_gui, m_pElem_conn, acTxt);
  
  // ------------------------------------------------
  // Periodically call GUIslice update function
  // ------------------------------------------------
  gslc_Update(&m_gui);
}
