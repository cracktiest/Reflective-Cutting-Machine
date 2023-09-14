#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.hpp>
#define I2CADDR 0X20
LiquidCrystal_I2C lcd(0x27, 16, 2);


//keypad i2cw
const int IR_RECEIVE_PIN = 10;
const int relay = 6;
const int sensorakhir = 5;
const int button = A0;
int dir;
//int pos;

//pid
long prevT, act = 0;
float eprev = 0;
float eintegral = 0;
float kp = 1;      //1;
float kd = 0.050;  //0.002;
float ki = 0.00;   //0.00;
long currT;
float deltaT;
long pos = 0;
const float setPoint = 22.81964;//23.04898;//22.81964;  //22.61724;//22.80286;//23.943;//23.61881;

int a, c, b, d = 0;
int menu = 1;
int nilaiakhir_ir = 0;
//volatile int counter=0;
volatile long counter, temp = 0;
volatile unsigned int temp2, counter2 = 0;  //This variable will increase or decrease depending on the rotation of encoder
#define PWM 7
#define PINA 8
#define PINB 9
unsigned int jumlah, proses, aencoder, bencoder, xencoder, yencoder, cencoder;
float uk, ukuran;



void menu1() {
  //  //Serial.print ("Menu Utama :");
  //  //Serial.println(menu);
  //  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Step = ");
  lcd.print(counter);
  //  lcd.print("PEMOTONG ELASTIS");
  lcd.setCursor(0, 1);
  lcd.print("<Back");
  lcd.print("      ");
  lcd.setCursor(11, 1);
  lcd.print("Next>");
  //  delay(100);
}

void menu2() {
  //Serial.println("Pilih Ukuran");
  ukuran = aencoder * 0.1;
  //  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PILIH UKURAN =  ");
  lcd.setCursor(0, 1);
  lcd.print(ukuran);
  lcd.print(" mm");
  lcd.print("                ");
}

void menu3() {
  //Serial.println("Pilih Jumlah");
  //  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PILIH JUMLAH = ");
  lcd.setCursor(0, 1);
  lcd.print(aencoder);
  lcd.print(" pcs");
  lcd.print("                ");
  proses = jumlah;
}

void menu4() {
  //Serial.print("Ukuran = "); //Serial.print(ukuran); //Serial.print(" Jumlah = "); //Serial.println(jumlah);
  //  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("UKURAN =");
  lcd.print(ukuran);
  lcd.print(" mm");

  //  uk = ukuran - 1;
  lcd.setCursor(0, 1);
  lcd.print("JUMLAH =");
  lcd.print(jumlah);
  lcd.print(" pcs     ");
  //  cencoder=(int(uk*langkahstep));
}

void menu5() {
  lcd.setCursor(0, 0);
  lcd.print("  KONFIRMASI ?  ");
  lcd.setCursor(0, 1);
  lcd.print("TEKAN OK =PROSES");
}

void (*Reset)(void) = 0;  // declare reset fuction at address 0


void kalibrasi() {
  if (counter != temp) {
    //Serial.print("counter=");
    //Serial.println(counter);
    temp = counter;
  }
}

void decode_repeat(int lastcode) {
  switch (lastcode) {
    case 1:
      analogWrite(PWM, 96);
      digitalWrite(PINA, LOW);
      digitalWrite(PINB, HIGH);
      delay(50);
      digitalWrite(PINA, LOW);
      digitalWrite(PINB, LOW);
      analogWrite(PWM, 0);
      //      counter = 0;
      break;
    case 2:

      analogWrite(PWM, 96);
      digitalWrite(PINA, HIGH);
      digitalWrite(PINB, LOW);
      delay(50);
      digitalWrite(PINA, LOW);
      digitalWrite(PINB, LOW);
      analogWrite(PWM, 0);
      break;
    default:
      break;
      //      counter = 0;
  }
}

void decodeIR()  // Indicate what key is pressed
{
  lcd.clear();
  nilaiakhir_ir = 0;
  //  Serial.println(IrReceiver.decodedIRData.decodedRawData); // Print "old" raw data
  switch (IrReceiver.decodedIRData.decodedRawData) {  // compare the value to the following cases
    case 001:
      //Serial.println("Tombol maju ditekan");
      analogWrite(PWM, 65);
      dir = 1;
      delay(100);
      dir = 2;
      analogWrite(PWM, 0);
      break;
    case 002:
      //Serial.println("Tombol mundur ditekan");
      analogWrite(PWM, 65);
      dir = -1;
      delay(100);
      dir = 2;
      analogWrite(PWM, 0);
    case 3125149440:  // if the value is equal to 0xFD00FF
      //Serial.println("Tombol 1 ditekan");
      if (aencoder == 0)
        aencoder = 1;
      else
        aencoder = (aencoder * 10) + 1;
      break;
    case 3108437760:
      //Serial.println("Tombol 2 ditekan");
      if (aencoder == 0)
        aencoder = 2;
      else
        aencoder = (aencoder * 10) + 2;
      break;
    case 3091726080:
      //Serial.println("Tombol 3 ditekan");
      if (aencoder == 0)
        aencoder = 3;
      else
        aencoder = (aencoder * 10) + 3;
      break;
    case 3141861120:
      //Serial.println("Tombol 4 ditekan");
      if (aencoder == 0)
        aencoder = 4;
      else
        aencoder = (aencoder * 10) + 4;
      break;
    case 3208707840:
      //Serial.println("Tombol 5 ditekan");
      if (aencoder == 0)
        aencoder = 5;
      else
        aencoder = (aencoder * 10) + 5;
      break;
    case 3158572800:
      //Serial.println("Tombol 6 ditekan");
      if (aencoder == 0)
        aencoder = 6;
      else
        aencoder = (aencoder * 10) + 6;
      break;
    case 4161273600:
      //Serial.println("Tombol 7 ditekan");
      if (aencoder == 0)
        aencoder = 7;
      else
        aencoder = (aencoder * 10) + 7;
      break;
    case 3927310080:
      //Serial.println("Tombol 8 ditekan");
      if (aencoder == 0)
        aencoder = 8;
      else
        aencoder = (aencoder * 10) + 8;
      break;
    case 4127850240:
      //Serial.println("Tombol 9 ditekan");
      if (aencoder == 0)
        aencoder = 9;
      else
        aencoder = (aencoder * 10) + 9;
      break;
    case 3860463360:
      //Serial.println("Tombol 0 ditekan");
      if (aencoder == 0)
        aencoder = 0;
      else
        aencoder = (aencoder * 10) + 0;
      break;
    case 3877175040:
      nilaiakhir_ir = 1;
      //Serial.println("Tombol up ditekan");
      analogWrite(PWM, 96);
      digitalWrite(PINA, LOW);
      digitalWrite(PINB, HIGH);
      delay(100);
      digitalWrite(PINA, HIGH);
      digitalWrite(PINB, LOW);
      analogWrite(PWM, 0);
      //      counter = 0;
      break;
    case 2907897600:
      nilaiakhir_ir = 2;
      //Serial.println("Tombol down ditekan");
      analogWrite(PWM, 96);
      digitalWrite(PINA, HIGH);
      digitalWrite(PINB, LOW);
      delay(100);
      digitalWrite(PINA, LOW);
      digitalWrite(PINB, LOW);
      analogWrite(PWM, 0);
      //      counter = 0;
      break;
    case 2774204160:
      //Serial.println("Tombol Next ditekan");
      menu++;
      delay(200);
      //Serial.print("Menu ke ="); //Serial.println(menu);
      break;
    case 4144561920:
      //Serial.println("Tombol Prev ditekan");
      lcd.clear();
      menu--;
      delay(200);
      //Serial.print("Menu ke ="); //Serial.println(menu);
      break;
    case 3810328320:  //ok
      //Serial.println("Tombol OK ditekan");
      if (menu == 1) {
        act = ukuran;
        menu = 3;
      } else if (menu == 2) {
        //Serial.println(ukuran);
        uk = ukuran;
        act = (uk * setPoint);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Step = ");
        lcd.print(act);
        lcd.setCursor(0, 1);
        lcd.print("SAVED");
        delay(1500);
        menu = 3;
        aencoder = 0;
      } else if (menu == 3) {
        //Serial.println("AAA");
        jumlah = aencoder;
        proses = jumlah;
        //Serial.println(jumlah);
        lcd.setCursor(11, 1);
        lcd.print("SAVED");
        delay(200);
        menu = 4;
        aencoder = 0;
      } else if (menu == 5) {
        counter = 0;
        for (jumlah; jumlah > 0; jumlah--) {
          lcd.clear();
          counter = 0;
          jalan();
          //Serial.println(jumlah);
        }
        if (jumlah == 0) {
          Reset();
          menu = 1;
          ukuran = 0;
          proses = 0;
        }
      }
      break;
    case 4061003520:
      lcd.setCursor(0, 0);
      lcd.print(" PROGRAM  RESET ");
      //      beep(500);
      Reset();
      break;
    case 3910598400:  //*
      if (menu == 5 || menu == 1) {
        RelayMainCut(500);
        counter = 0;
      }
      //      beep(500);
      //      return setup();
      break;

    case 0:
      decode_repeat(nilaiakhir_ir);
      //    beep(200);
      break;

    default:
      break;
  }
  if (menu > 5 || menu < 1) {
    lcd.clear();
    menu = 1;
    delay(200);
  }
}

void RelayMainCut(int durasirelay) {
//  digitalWrite(13, HIGH);
  //  delay(500);
//  digitalWrite(13, LOW);
  digitalWrite(relay, LOW);
  delay(durasirelay);  // Wait 1.5 second
  digitalWrite(relay, HIGH);
   delay(durasirelay / 2);
  lcd.init();
  lcd.clear();
//  digitalWrite(13, HIGH);
  //  delay(100);
//  digitalWrite(13, LOW);
}

void setup() {
  Wire.begin();
  lcd.init();
  Serial.begin(9600);  //serial plotter = 9600, debug 115200
  //Serial.println("Halo");
  //Serial.print("C =");
  //Serial.println(c);
  pinMode(2, INPUT_PULLUP);  // internal pullup input pin 2
  pinMode(3, INPUT_PULLUP);
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);
  attachInterrupt(1, ai1, RISING);

  lcd.backlight();
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);  // Start the receiver
  //  pinMode(PINA, OUTPUT);
  pinMode(sensorakhir, INPUT);
  pinMode(PINB, OUTPUT);
  pinMode(relay, OUTPUT);
  //  pinMode(SPEED,OUTPUT);
  //  pinMode(button, INPUT_PULLUP);
  //  pinMode(countera, INPUT_PULLUP);
  digitalWrite(PINA, LOW);
  digitalWrite(PINB, LOW);
  //  analogWrite(SPEED,60);
  digitalWrite(relay, HIGH);
  //  attachInterrupt(digitalPinToInterrupt(countera), a, RISING);
  //  while(c>=2){
  //  }
}

void jalan() {

  //Serial.println("Jalan");
  lcd.setCursor(0, 0);
  lcd.print(" MESIN BERJALAN ");
  lcd.setCursor(0, 1);
  lcd.print(jumlah);
  lcd.print('/');
  lcd.print(proses);
  //    posi=0;
  long currT;
  float deltaT;
  int e;
  float pwr;
  c, d, counter, pos = 0;
  for (int osi = 20; osi >= 5;) {
    currT = micros();
    deltaT = ((float)(currT - prevT)) / (1.0e6);
    prevT = currT;
    // Read the position
    pos = 0;
    noInterrupts();  // disable interrupts temporarily while reading
    pos = counter;
    interrupts();  // turn interrupts back on
    // error
    long e = pos - act;
    // derivative
    float dedt = (e - eprev) / (deltaT);
    // integral
    eintegral = eintegral + e * deltaT;
    // control signal
    float u = kp * e + kd * dedt + ki * eintegral;
    // motor power
    pwr = fabs(u);
    if (pwr > 255) {
      pwr = 255;
    } else if (pwr <= 65 && pwr >= 3) {
      pwr = 70;
    }
    // motor direction
    int dir = 1;
    if (u < 0) {
      dir = -1;
    }

    if ((act - pos) > -5 && (act - pos) < 5) {
      osi--;
      dir = 2;
    } else {
      osi++;
    }
    // signal the motor
    setMotor(dir, pwr, PWM, PINA, PINB);

    // store previous error
    eprev = e;
    //      Serial.flush();
    //Serial.print(act);
    //Serial.print(" ");
    //Serial.print(pos);
    //Serial.println();

    Serial.print(act);
    Serial.print(" ");
    Serial.print(pos);
     Serial.print(" ");
     Serial.print(pwr);
    Serial.println();
  }
  int dir = 2;
  setMotor(dir, pwr, PWM, PINA, PINB);
  //    analogWrite(SPEED,0);
  //    for(counter=0; counter <=act;){
  //      digitalWrite(PINB,HIGH);
  //      calc=map(counter,0,act,100,53);
  //      analogWrite(SPEED,calc);
  ////      fade=counter;
  //      //Serial.println(calc);
  //    }
  //read-me jika mau debug maka pos dan counter ditempatkan sebelum delay 500

  //Serial.print("Step ="); //Serial.print(counter);
  //Serial.print('/'); //Serial.println(act);
  //Serial.print("Selisih =");
  //Serial.println(counter - act);
  // delay(100);
  RelayMainCut(200);
  // if(digitalRead(sensorakhir)== LOW){
  //   RelayMainCut(350);
  // }
  // else{
  // }
  while (digitalRead(sensorakhir) == LOW) {
    RelayMainCut(200);
  }
  digitalWrite(relay, HIGH);
  pos, counter = 0;
}

void loop() {

  if (IrReceiver.decode()) {
    decodeIR();
    IrReceiver.resume();
  }
  //  kalibrasi();
  //  tombol();
  switch (menu) {
    case 1:  //menu utama,
      menu1();
      c = 0;
      jumlah, ukuran, a = 0;
      break;
    case 2:  //pilih ukuran elastis yang dipotong
      menu2();
      break;
    case 3:  //pilih jumlah yang akan dipotong
      menu3();
      break;
    case 4:  //konfirmasi
      menu4();
      break;
    case 5:  //proses
      menu5();
      break;
  }

  if (counter != temp) {
    temp = counter;
  }
}


void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(3) == LOW) {
    counter--;
  } else {
    counter++;
  }
}


void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if (digitalRead(2) == LOW) {
    counter++;
  } else {
    counter--;
  }
}



void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(PWM, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}
