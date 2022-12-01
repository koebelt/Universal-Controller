#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/************************** Radio Declarations ********************************/

#define pinCE 7
#define pinCSN 8
RF24 radio(pinCE, pinCSN, 4000000);
#define tunnel1 "PIPE1"
#define tunnel2 "PIPE2"
#define tunnel3 "PIPE3"
#define tunnel4 "PIPE4"
#define tunnel5 "PIPE5"
#define tunnel6 "PIPE6"
const byte adresses[][6] = {tunnel1, tunnel2, tunnel3, tunnel4, tunnel5, tunnel6};
bool connected = false;
bool wasConnected = false;
int currentChannel = 0;
int connectedChannel = -1;
int sentPackets = 0;
int receivedPackets = 0;
int lostPackets = 0;
int recievedPacketsPercent = 0;

/********************* End of Radio Declarations ******************************/

/*********************** Joysticks Declaration ********************************/

#define pinJoy1X A0
#define pinJoy1Y A1
#define pinJoy2X A2
#define pinJoy2Y A3

/********************* End of Joysticks Declaration ***************************/


/*********************** Buttons Declaration **********************************/

#define pinButton1 2
#define pinButton2 3
#define pinButton3 4
#define pinButton4 5
// #define pinButton5 6

/********************* End of Buttons Declaration *****************************/

/*********************** LCD Display Declaration ******************************/

LiquidCrystal_I2C lcd(0x27, 20, 4);
unsigned long startMillis;
unsigned long currentMillis;

/********************* End of LCD Display Declaration *************************/

typedef struct joystick {
  int x;
  int y;
} joystick_t;

typedef struct remoteData {
  joystick_t ljoystick;
  joystick_t rjoystick;
  byte button1;
  byte button2;
  byte button3;
  byte button4;
} remoteData_t;

remoteData_t remoteData;

char message[32];

void listener(void) {
  radio.startListening();
  if (radio.available()) {
    while (radio.available())
    {
      radio.read(&message, sizeof(message));
      Serial.println("Received data");
      Serial.println(message);
    }
    delay(20);
  } // radio.startListening();
  // if (radio.available()) {
  //   message = (char *)malloc(sizeof(char) * 32);
  //   radio.read(&message, sizeof(message));
  // }
  // radio.stopListening();
  // return message;
}

bool send()
{
  return radio.write(&remoteData, sizeof(remoteData));
}

void readJoysticks() {
  (analogRead(pinJoy1X) >= 450 and analogRead (pinJoy1X) <= 600) ?
  remoteData.ljoystick.x = 525 : remoteData.ljoystick.x = analogRead(pinJoy1X);
  (analogRead(pinJoy1Y) >= 450 and analogRead(pinJoy1Y) <= 600) ?
  remoteData.ljoystick.y = 525 : remoteData.ljoystick.y = analogRead(pinJoy1Y);
  (analogRead(pinJoy2X) >= 450 and analogRead(pinJoy2X) <= 600) ?
  remoteData.rjoystick.x = 525 : remoteData.rjoystick.x = analogRead(pinJoy2X);
  (analogRead(pinJoy2Y) >= 450 and analogRead(pinJoy2Y) <= 600) ?
  remoteData.rjoystick.y = 525 : remoteData.rjoystick.y = analogRead(pinJoy2Y);
  // remoteData.ljoystick.y = analogRead(pinJoy1Y);
  // remoteData.rjoystick.x = analogRead(pinJoy2X);
  // remoteData.rjoystick.y = analogRead(pinJoy2Y);
}

void readButtons() {
  remoteData.button1 = digitalRead(pinButton1);
  remoteData.button2 = digitalRead(pinButton2);
  remoteData.button3 = digitalRead(pinButton3);
  remoteData.button4 = digitalRead(pinButton4);
  // remoteData.button5 = digitalRead(pinButton5);
}

void displayLCD() {
  currentMillis = millis();
  if (currentMillis - startMillis >= 500)
  {
    lcd.clear();
    if (!wasConnected) {
      lcd.setCursor(0, 0);
      lcd.print("Connecting...");
      lcd.setCursor(0, 1);
      lcd.print("Channel: ");
      lcd.print(currentChannel);
    } else {
      lcd.setCursor(0, 0);
      lcd.print("Connected");
      lcd.setCursor(17, 0);
      if (recievedPacketsPercent >= 70) {
        lcd.print(".il");
      } else if (recievedPacketsPercent > 55) {
        lcd.print(".i");
      } else if (recievedPacketsPercent > 30) {
        lcd.print(".");
      } else {
        lcd.print("");
      }
      // lcd.setCursor(0, 1);
      // lcd.print("Receiving data: ");
      lcd.setCursor(0, 1);
      lcd.print(message);
    }
    lcd.display();
    startMillis = currentMillis;
  }
}

void setup() {
  Serial.begin(9600);

  
  /* Radio initialisation */
  if (!radio.begin())
    Serial.println("Radio initialisation failed");
  else
    Serial.println("Radio initialisation success");
  // radio.setChannel(currentChannel);
  radio.openWritingPipe(adresses[0]);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, adresses[1]);
  radio.openReadingPipe(2, adresses[2]);
  radio.openReadingPipe(3, adresses[3]);
  radio.openReadingPipe(4, adresses[4]);
  radio.openReadingPipe(5, adresses[5]);
  radio.stopListening();
  /* End of Radio Initialisation */

  /* Joysticks initialisation */
  pinMode(pinJoy1X, INPUT);
  pinMode(pinJoy1Y, INPUT);
  pinMode(pinJoy2X, INPUT);
  pinMode(pinJoy2Y, INPUT);
  /* End of Joysticks initialisation */

  /* Buttons initialisation */
  pinMode(pinButton1, INPUT);
  pinMode(pinButton2, INPUT);
  pinMode(pinButton3, INPUT);
  pinMode(pinButton4, INPUT);
  remoteData.button1 = HIGH;
  remoteData.button2 = HIGH;
  remoteData.button3 = HIGH;
  remoteData.button4 = HIGH;
  // pinMode(pinButton5, INPUT);
  /* End of Buttons initialisation */

  /* LCD Display initialisation */
  lcd.init();
  lcd.backlight();
  lcd.setDelay(0, 10);
  startMillis = millis();
  // lcd.backlight();
  // lcd.setCursor(1, 0);
  // lcd.print("HELLO");
  // lcd.setCursor(8, 1);
  // lcd.print("WORLD");

  /* End of LCD Display initialisation */
}

void loop() {
  if (!connected) {
    if (connectedChannel == -1) {
      currentChannel += 1;
      if (currentChannel > 125) {
        currentChannel = 0;
      }
      radio.setChannel(currentChannel);
    }
  } else {
    connectedChannel = currentChannel;
    listener();
    delay(5);
    // char *recieved = listener();

    // if (recieved != nullptr) {
    //   Serial.println(recieved);
    //   free(recieved);
    // }
  }
  readJoysticks();
  readButtons();
  displayLCD();
  radio.stopListening();
  delay(5);
  if (!send()) {
    connected = false;
    sentPackets += 1;
    // lostPackets = sentPackets - receivedPackets;
    recievedPacketsPercent = (receivedPackets * 100) / sentPackets;
    if (recievedPacketsPercent < 15) {
      connected = false;
      sentPackets = 0;
      receivedPackets = 0;
      wasConnected = false;
    } if (sentPackets >= 100) {
      receivedPackets = 0;
      sentPackets = 0;
    }
  } else {
    sentPackets += 1;
    receivedPackets += 1;
    connected = true;
    wasConnected = true;
  }
  // Serial.println("Hello");
  // lcd.backlight();
  // lcd.setCursor(0,0);
  // lcd.print("aled");
  // lcd.setCursor(0,1);
  // lcd.print("zebi");
  // lcd.setCursor(0,2);
  // lcd.print("panda");
  // lcd.setCursor(0,3);
  // lcd.print("carré");

  // if (radio.write(&message, sizeof(message))) // Envoi de notre message
  //   connected = true;
  // else
  //   connected = false;
}