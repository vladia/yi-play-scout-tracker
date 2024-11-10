#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
#define IS_DEBUG 0
#if IS_DEBUG
   #define Serial SerialUSB
#endif   
#endif

//#define DEVICE_MODE  MODE_TRACKER_RECEIVER
//#define DEVICE_MODE  MODE_TRACKER_SENDER
//#define DEVICE_MODE  mode;
//#define DEVICE_COLOR device_color
#define DEVICE_REPEAT_PERIOD 1000   // Repeat  period for sender

byte device_color;

typedef enum {
  MODE_TRACKER_SENDER = 0,
  MODE_TRACKER_RECEIVER,  
  MODE_INDEPENDENT_NODE,    
} tracker_mode_t;

tracker_mode_t device_mode = MODE_TRACKER_SENDER;

#define ARDUINO_SAMD_ZERO // Definition needed for FastLed Library

#define ANALOG_RES 12 // Analog resolution

#define BUZZER_PIN    5
#define BUZZER_FREQ   4000

#include <Wire.h>
#include <RFM69.h>
#include <FastLED.h>

const struct CRGB colors[] = {
CRGB::Red,
CRGB::Green,
CRGB::Blue,
CRGB::Yellow,
CRGB::Cyan,
CRGB::Violet,
CRGB::OrangeRed,
CRGB::White,
};


/* 
 * RFM69 settings 
 */
#define NETWORKID   110
#define NODEID      100
#define GATEWAYID   200
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

#define SIGNAL_COLOR colors[device_color & 0x07]
#define SIGNAL_BRIGHTNESS 32
#define SIGNAL_MIN -100
#define SIGNAL_MAX -20

void display_rssi(int rssi, struct CRGB color)
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
      leds_out[NUM_LEDS_OUT - i - 1] = color;
      leds_out[NUM_LEDS_OUT - i - 1].nscale8((segments - (strength % segments)) * (255 / segments) );        
    }
    if (i > (strength / segments))
    {
      leds_out[NUM_LEDS_OUT - i - 1] = color;   
    }
  }
  FastLED.setBrightness(SIGNAL_BRIGHTNESS);  
  FastLED.show();  
}

#define BTN_SW3    30
#define BTN_SW5    31

#define SUB_EN    38

#define COLOR0    15
#define COLOR1    18
#define COLOR2    17
#define COLOR3    16

#define MODE0   1
#define MODE1   0
#define MODE2   4
#define MODE3   2

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
  
  //for (;;) {

#if 1
  pinMode(COLOR0, INPUT_PULLUP);
  pinMode(COLOR1, INPUT_PULLUP);
  pinMode(COLOR2, INPUT_PULLUP);
  pinMode(COLOR3, INPUT_PULLUP);
  device_color = 0x0F ^ ((digitalRead(COLOR3) << 3) | (digitalRead(COLOR2) << 2) | (digitalRead(COLOR1) << 1) | digitalRead(COLOR0));
  pinMode(COLOR0, INPUT);
  pinMode(COLOR1, INPUT);
  pinMode(COLOR2, INPUT);
  pinMode(COLOR3, INPUT);


  digitalWrite(SUB_EN, HIGH);
  delay(1);        
  pinMode(MODE0, INPUT_PULLUP);
  pinMode(MODE1, INPUT_PULLUP);
  pinMode(MODE2, INPUT_PULLUP);
  pinMode(MODE3, INPUT_PULLUP);
  device_mode = (tracker_mode_t)(0x0F ^ ((digitalRead(MODE3) << 3) | (digitalRead(MODE2) << 2) | (digitalRead(MODE1) << 1) | digitalRead(MODE0)));
  pinMode(MODE0, INPUT);
  pinMode(MODE1, INPUT);
  pinMode(MODE2, INPUT);
  pinMode(MODE3, INPUT);
  digitalWrite(SUB_EN, LOW);  
#else  
  device_color = 1;
  device_mode = MODE_INDEPENDENT_NODE;
#endif  

#if IS_DEBUG
  Serial.print(device_color);
  Serial.print(" ");
  Serial.println(device_mode);  
#endif  

  //delay(100);        
  //}

  digitalWrite(SUB_EN, LOW);

  pinMode(BTN_SW3, INPUT);
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
case MODE_INDEPENDENT_NODE:
      radio.initialize(FREQUENCY,device_color,NETWORKID);

      FastLED.addLeds<LED_TYPE, LED_PIN_OUT, COLOR_ORDER>(leds_out, NUM_LEDS_OUT).setCorrection( TypicalLEDStrip );
      for (int i=0; i<NUM_LEDS_OUT; i++)
      {
          leds_out[i] = CRGB::Black;          
      }
      
      break;
    default:      
      break;
  }
}

#if IS_DEBUG
byte ackCount=0;
uint32_t packetCount = 0;
#endif

byte last_input = 0, is_sound = 0;
int enabled = 0, last_enabled = 0;

void loop() {


   switch (device_mode) {
   case MODE_TRACKER_RECEIVER:
   {
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
            pinMode(BUZZER_PIN, OUTPUT);
            tone(BUZZER_PIN, BUZZER_FREQ);
            // Wait until it settles
            delay(10);
            // Display RSSI
            display_rssi(radio.RSSI, SIGNAL_COLOR); 
            // Wait for some time
            delay(300);
            noTone(BUZZER_PIN);
            pinMode(BUZZER_PIN, INPUT);
            // Disable 5V power supply to save power
            digitalWrite(SUB_EN, LOW); 
            digitalWrite(LED_PIN_OUT, LOW);
         }
         break;
   }
   case MODE_TRACKER_SENDER:
   {
        unsigned char data[2];
        
        Blink(PIN_LED2,10);
        
        radio.send(NODEID, data, sizeof(data), 0);
        radio.sleep();
        delay(DEVICE_REPEAT_PERIOD);        
        break;
   }    
   case MODE_INDEPENDENT_NODE:
   {
        unsigned char data[2];        
        byte input = digitalRead(BTN_SW3);
        
        if ((input == 0)/* && (last_input == 1)*/)
        {          
            if (!is_sound)
            {
              digitalWrite(SUB_EN, HIGH);
              pinMode(BUZZER_PIN, OUTPUT);
              tone(BUZZER_PIN, BUZZER_FREQ);            
                  delay(10);
                  // Display RSSI
                  display_rssi(SIGNAL_MAX, colors[device_color & 0x07]); 

              
              is_sound = 1;
            }
            
            Blink(PIN_LED2,10);            
            data[0] = input;
            radio.send(RF69_BROADCAST_ADDR, data, sizeof(data), 0);                      
            radio.sleep();
            delay(30);
        } else
        if ((input == 1) && ((last_input & 0x07) != 0x07))
        {
            if (is_sound)
            {
               noTone(BUZZER_PIN);
               pinMode(BUZZER_PIN, INPUT);
               // Disable 5V power supply to save power
               digitalWrite(SUB_EN, LOW); 
               is_sound = 0;
            }           
             Blink(PIN_LED2,10);            
             data[0] = input;
             radio.send(RF69_BROADCAST_ADDR, data, sizeof(data), 0);          
             radio.sleep();          
             delay(30);            
       } else {
       
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

              if (radio.DATA[0] == 0)
              {  
                if (enabled==0)
                {
                  // Enable 5V power supply
                  digitalWrite(SUB_EN, HIGH);
                  pinMode(BUZZER_PIN, OUTPUT);
                  tone(BUZZER_PIN, BUZZER_FREQ);
                  // Wait until it settles
                  delay(10);
                  // Display RSSI
                  display_rssi(radio.RSSI, colors[radio.SENDERID & 0x07]); 
                }
                enabled = 1;                                  
              } else
              if (radio.DATA[0] != 0)
              {
                if (enabled)
                {                
                // Wait for some time
//                delay(300);
                  noTone(BUZZER_PIN);
                  pinMode(BUZZER_PIN, INPUT);
                  // Disable 5V power supply to save power
                  digitalWrite(SUB_EN, LOW); 
                  digitalWrite(LED_PIN_OUT, LOW);
                  enabled = 0;                  
                }
              }
           }
           enabled <<= 1;

           if ((enabled == 0)&&(last_enabled != 0))
           {
              // Wait for some time
//              delay(300);
               noTone(BUZZER_PIN);
               pinMode(BUZZER_PIN, INPUT);
               // Disable 5V power supply to save power
               digitalWrite(SUB_EN, LOW); 
               digitalWrite(LED_PIN_OUT, LOW);
               enabled = 0;
           }
           
           last_enabled = enabled;
        }
        last_input <<= 1;
        last_input |= input;
//        last_input1 = input;
        break;
   }
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
