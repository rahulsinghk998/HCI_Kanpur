int datatype;
int accx[3];
char voicecommand;

void setup() {
  // put your setup code here, to run once:
Serial2.begin(9600);
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("hello");
  delay(1000);
if(Serial2.available())
    {
      datatype = Serial2.read();
      //Serial.print(datatype);
      
      if(datatype == 'A'){
        while(!Serial2.available());
        accx[0]= Serial2.read();
         while(!Serial2.available());
        accx[1]= Serial2.read();
        while(!Serial2.available());
        accx[2]= Serial2.read();
        Serial.print("AccX[0]: ");
        Serial.println((int)accx[0]);
        Serial.print("AccX[1]: ");
        Serial.println((int)accx[1]);
        Serial.print("AccX[2]: ");
        Serial.println((int)accx[2]);
        Serial.print('\n');
      }
      else if(datatype == 'V'){
        while(!Serial2.available());
        voicecommand = Serial2.read();
        Serial.print(voicecommand);
      }
      
    }
    
     
    
}

//
//
//void setup() {
//  // initialize both serial ports:
//  Serial.begin(9600);
//  Serial1.begin(9600);
//}
//
//void loop() {
//  // read from port 1, send to port 0:
//  if (Serial1.available()) {
//    int inByte = Serial1.read();
//    Serial.write(inByte);
//  }
//
//  // read from port 0, send to port 1:
//  if (Serial.available()) {
//    int inByte = Serial.read();
//    Serial1.write(inByte);
//  }
//}
