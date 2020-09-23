
float speedvalserial;
int speedval;
void setup() {
  // put your setup code here, to run once:

pinMode(11,OUTPUT);
pinMode(9,OUTPUT);
Serial.begin(9600);
}
void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()>0){
    speedvalserial=Serial.parseFloat();
//    if(speedvalserial==1.0){
//      speedvalserial=0.9999;
//    }
    speedval=speedvalserial*255;
    analogWrite(11,speedval);
    analogWrite(9,speedval);
  }
}
