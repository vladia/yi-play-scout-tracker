#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
#if IS_DEBUG
   #define Serial SerialUSB
#endif   
#endif

#define DEVICE_MODE  MODE_TRACKER_RECEIVER
//#define DEVICE_MODE  MODE_TRACKER_SENDER
#define DEVICE_COLOR CRGB::Yellow
#define DEVICE_REPEAT_PERIOD 1000   // Repeat  period for sender

typedef enum {
  MODE_TRACKER_SENDER,
  MODE_TRACKER_RECEIVER,  
} tracker_mode_t;

tracker_mode_t device_mode = DEVICE_MODE;

#define ARDUINO_SAMD_ZERO // Definition needed for FastLed Library

#define ANALOG_RES 12 // Analog resolution

#include <Wire.h>
#include <RFM69.h>
#include <FastLED.h>


/* 
 * RFM69 settings 
 */
#define NETWORKID   110
#define NODEID      1
#define GATEWAYID   2
#define FREQUENCY     RF69_868MHZ
#define IS_RFM69HW true   //uncomment only for RFM69HW! Leave out if you have RFM69W!

#define RFM69_NSS           8
#define RFM69_INTERRUPT_PIN 3
#define RFM69_INTERRUPT_NUM digitalPinToInterrupt(RFM69_INTERRUPT_PIN)

RFM69 radio(RFM69_NSS, RFM69_INTERRUPT_PIN, IS_RFM69HW, RFM69_INTERRUPT_NUM);

/*
 * FastLED settings
 */
#define LED_PIN_OUT     14
#define NUM_LEDS_OUT    10
CRGB leds_out[NUM_LEDS_OUT];
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB 

#define SIGNAL_COLOR DEVICE_COLOR
#define SIGNAL_BRIGHTNESS 32
#define SIGNAL_MIN -100
#define SIGNAL_MAX -20

void display_rssi(int rssi)
{
  int points = SIGNAL_MAX - SIGNAL_MIN;
  int segments = points /  NUM_LEDS_OUT;
  int strength = SIGNAL_MAX - rssi;

  char buff[50];
  sprintf(buff, "\n%d %d %d %d\n", points, segments, strength, strength % segments);
//  Serial.println(buff);

  
  for (int i=0; i<NUM_LEDS_OUT; i++)
  {
    if (i < (strength / segments))
    {
      leds_out[NUM_LEDS_OUT - i - 1] = CRGB::Black;          
    }
    if (i == (strength / segments))    
    {
      leds_out[NUM_LEDS_OUT - i - 1] = SIGNAL_COLOR;
      leds_out[NUM_LEDS_OUT - i - 1].nscale8((segments - (strength % segments)) * (255 / segments) );        
    }
    if (i > (strength / segments))
    {
      leds_out[NUM_LEDS_OUT - i - 1] = SIGNAL_COLOR;   
    }
  }
  FastLED.setBrightness(SIGNAL_BRIGHTNESS);  
  FastLED.show();  
}

#define BTN_SW3    30
#define BTN_SW5    31

#define SUB_EN    38

void setup()
{

#if IS_DEBUG
  Serial.begin(115200);
//  while (!Serial);
#endif

  pinMode(PIN_LED1, OUTPUT);
  digitalWrite(PIN_LED1, HIGH);
  pinMode(PIN_LED2, OUTPUT);
  digitalWrite(PIN_LED2, HIGH);
  pinMode(SUB_EN, OUTPUT);
  digitalWrite(SUB_EN, LOW);

//  pinMode(BTN_SW3, INPUT);
//  pinMode(BTN_SW5, INPUT);

  switch (device_mode) {
    case MODE_TRACKER_RECEIVER:
      // RFM69 INIT
      radio.initialize(FREQUENCY,NODEID,NETWORKID);
#if IS_DEBUG
      char buff[50];
      sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
      Serial.println(buff);
#endif

//      digitalWrite(SUB_EN, HIGH);
      FastLED.addLeds<LED_TYPE, LED_PIN_OUT, COLOR_ORDER>(leds_out, NUM_LEDS_OUT).setCorrection( TypicalLEDStrip );
      for (int i=0; i<NUM_LEDS_OUT; i++)
      {
          leds_out[i] = CRGB::Black;          
      }
      //FastLED.setBrightness(32);  
/*      FastLED.show();
      digitalWrite(SUB_EN, LOW);    */
      break;
case MODE_TRACKER_SENDER:
      // RFM69 INIT
      radio.initialize(FREQUENCY,GATEWAYID,NETWORKID);

      break;
    default:      
      break;
  }
}

#if IS_DEBUG
byte ackCount=0;
uint32_t packetCount = 0;
#endif

void loop() {


   switch (device_mode) {
   case MODE_TRACKER_RECEIVER:
        if (radio.receiveDone())
        {  
            Blink(PIN_LED1,10);

#if IS_DEBUG
            Serial.print("#[");
            Serial.print(++packetCount);
            Serial.print(']');
            Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
            for (byte i = 0; i < radio.DATALEN; i++)
                Serial.print((char)radio.DATA[i]);
            Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");
#endif      
            if (radio.ACKRequested())
            {
                byte theNodeID = radio.SENDERID;
                radio.sendACK();
#if IS_DEBUG                
                Serial.print(" - ACK sent.");
#endif    
            }              
#if IS_DEBUG
            Serial.println();
#endif

            // Enable 5V power supply
            digitalWrite(SUB_EN, HIGH);
            // Wait until it settles
            delay(10);
            // Display RSSI
            display_rssi(radio.RSSI); 
            // Wait for some time
            delay(300);
            // Disable 5V power supply to save power
            digitalWrite(SUB_EN, LOW); 
            digitalWrite(LED_PIN_OUT, LOW);
         }
         break;
   case MODE_TRACKER_SENDER:
        unsigned char data[2];
        
        Blink(PIN_LED2,10);
        
        radio.send(NODEID, data, sizeof(data), 0);
        radio.sleep();
        delay(DEVICE_REPEAT_PERIOD);        
        break;
   default:
         break;
   }
   delay(10);  
}

void Blink(byte PIN, int DELAY_MS)
{
  digitalWrite(PIN,LOW);
  delay(DELAY_MS);
  digitalWrite(PIN,HIGH);
}
