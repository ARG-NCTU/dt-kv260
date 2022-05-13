#include <SoftwareSerial.h>
SoftwareSerial mySerial(3, 2); // RX, TX


int in1 =8; //The enabling of L298PDC motor driver board connection to the digital interface port 5
int in2 =9; //The enabling of L298PDC motor driver board connection to the digital interface port 4
int ena =5; 
int in3 =10; //The enabling of L298PDC motor driver board connection to the digital interface port 5
int in4 =11; //The enabling of L298PDC motor driver board connection to the digital interface port 4
int enb =6; 

char symbol_r='1',symbol_l='1';
int val_r=0, val_l=0;
String incomingString ="10001000\n";


void UART_decoder() //charToInt()(定义)
{
  symbol_r = incomingString[0];
  Serial.write(incomingString[0]);
  Serial.print("\n");
  
  int tmp =0;
  for(int i=1;i<4;i++)
  {
    Serial.write(incomingString[i]);
    Serial.print("\n");
    tmp = tmp * 10 + (incomingString[i] - '0');
  }
  val_r = tmp;

  symbol_l = incomingString[4];
  Serial.write(incomingString[4]);
  Serial.print("\n");
  int tmp1 =0;
  for(int i=5;i<8;i++)
  {
    Serial.write(incomingString[i]);
    Serial.print("\n");
    tmp1 = tmp1 * 10 + (incomingString[i] - '0');
  }
  val_l = tmp1;

  Serial.print("symbol_r:");
  Serial.write(symbol_r);
  Serial.print("val_r:");
  Serial.write(val_r);
  Serial.print(" ,symbol_l:");
  Serial.write(symbol_l);
  Serial.print("val_l:");
  Serial.write(val_l);
  Serial.print("\n");
  
}

void control(){
  if(symbol_r=='1'){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    analogWrite(ena,val_r);
  }
  if(symbol_r=='0'){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    analogWrite(ena,val_r);
  }
  if(symbol_l=='1'){
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    analogWrite(enb,val_l);
  }
  if(symbol_l=='0'){
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
    analogWrite(enb,val_l);
  }
}

void setup()
{
   Serial.begin(9600);  
   mySerial.begin(9600); 

   pinMode(in1, OUTPUT);   //L298P Control port settings DC motor driver board for the output mode
   pinMode(in2, OUTPUT);
   pinMode(ena, OUTPUT);
   pinMode(in3, OUTPUT);   //L298P Control port settings DC motor driver board for the output mode
   pinMode(in4, OUTPUT);
   pinMode(enb, OUTPUT);

   control();
   Serial.print("INIT DONE");
}



void loop()
{     
  while (!mySerial.available()) {}  //直到暫存器出現訊號才跳出迴圈
  incomingString = mySerial.readStringUntil('\n');
  UART_decoder();
  control();
}
