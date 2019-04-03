#define PULSE_PIN 12
unsigned long trm; 
const float pi = 3.14;
const float rayon=0.335; //rayon de la roue
float perimetre=2*rayon*pi; // 2m par tour de roue environ
float kmh=0;
float ms_1=0;

void setup() {
pinMode(PULSE_PIN, INPUT_PULLUP);
Serial.begin(9600);
}

void loop() {
  trm = pulseIn(PULSE_PIN, LOW);
  trm = (60 / (trm / 1000000.0));
  kmh = trm*perimetre*60/1000;
  ms_1= trm*perimetre/60;
  
  Serial.print(kmh);
 Serial.println("km/h"); 
  Serial.print(ms_1);
Serial.println("m/s");
   delay(500);
}
