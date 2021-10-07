void setup() {
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
}

void loop() {
  String x = String(analogRead(A0)); //blue
  String y = String(analogRead(A1)); //yellow
  String z = String(analogRead(A2)); //green
  String send_val = "x"+x+"y"+y+"z"+z;

  Serial.println(send_val); Serial.flush();
  delay(100);
}
