void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  analogReadResolution(16);
}
float val1;
float val2;
float avg = 0 ;
float cons = 21128;
float mult = 100.0/60.0;
float saved_val = 0;
#define n 500.0

void loop() {
  val1 =0;
  
  for (int i = 0; i<n; i++){
    val1 = val1 + analogRead(A0);
  }
  val1 = val1/n;
  avg = avg*0.93 +  val1*0.07;
  Serial.println((avg-cons)*mult);
}
