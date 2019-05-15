#include <FlexiTimer2.h>
#include <mcp_can.h>
#include <SPI.h>

// GPS Port D10
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
static const int RXPin = 10, TXPin = 11;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
unsigned char buffer[64];                   
int count=0;  

//Déclaration variables
const int SPI_CS_PIN = 53;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin
char carac;
int swap=0;
int taille=0;
char messagee[5];
int BP=0;



const int currentSensorPin = A2;  //Capteur intensité 
const int mVperAmp = 100;         //use 185 for 5A Module, and 66 for 30A Module
const int capt = A0;              //Capteur tension
const int BP_resetD = 2;           //Bouton reset distance
const int BP_stop = 8;            //Bouton Stop
float Vcapteur=0;     
float Vmesure=0;
float Vref  = 0;                  //read your Vcc voltage,typical voltage should be 5000mV(5.0V)
String o="";
bool race_started;                          
int ms=0;
int msgtaille=0;
int compteur=0;
const int BP_6 = 6;
int BPR;
String message;
int baterie;         
float CurrentValue;            
int rep=0;    
int tourprog;
int distance;
int tour;
const float pi=3.14;
const float D=0.7;

volatile float  signalDetectedMillis=0;
volatile boolean  signalDetected=false;
float t0=0;
float prev_vkmh=0;
float vkmh_flemme;
          
 void macro_temp(void)
{                          //sous programme

  compteur++;

}

void setup()

{   
     while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k

    {
       Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }

    Serial.println("CAN BUS Shield init ok!");
    //pinMode pour les 2 boutons
    pinMode(2, INPUT);
    pinMode(4, INPUT);
    pinMode(8, INPUT);

    //Serial pour la transmission sans fil
    Serial.begin(9600);       //Serial 
    ss.begin(GPSBaud);          //GPS
    Serial1.begin(19200);       //Serial antennes
    Vref = readVref();          //read the reference votage(default:VCC)
    pinMode(BP_6, INPUT);
    FlexiTimer2::set(200,macro_temp);
    distance=126;
    Serial3.begin(9600);
    tour = -1;
    //Vitesse
    pinMode(3,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(3),isr,FALLING);
    
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
long readVref()

{
    long result;
    #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
        ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_AT90USB1286__)
        ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
        ADCSRB &= ~_BV(MUX5);  
    #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
        ADMUX = _BV(MUX5) | _BV(MUX0);
    #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
        ADMUX = _BV(MUX3) | _BV(MUX2);
    #endif
    #if defined(__AVR__)
        delay(2);                                        
        ADCSRA |= _BV(ADSC);                             
        while (bit_is_set(ADCSRA, ADSC));
            result = ADCL;
            result |= ADCH << 8;
            result = 1126400L / result;  
        return result;
    #elif defined(__arm__)
        return (3300);                                  
    #else
        return (3300);                                 
    #endif
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
  void isr(void){
  signalDetectedMillis=millis();
  signalDetected=true;
  }

  float readDCCurrent(int Pin)
{
    int analogValueArray[31];
    for(int index=0;index<31;index++)
    {
      analogValueArray[index]=analogRead(Pin);
    }

    int i,j,tempValue;
    for (j = 0; j < 31 - 1; j ++)
    {

        for (i = 0; i < 31 - 1 - j; i ++)
        {
            if (analogValueArray[i] > analogValueArray[i + 1])
            {
                tempValue = analogValueArray[i];
                analogValueArray[i] = analogValueArray[i + 1];
                analogValueArray[i + 1] = tempValue;
            }

        }

    }

    float medianValue = analogValueArray[(31 - 1) / 2];
    float DCCurrentValue = (medianValue / 1024.0 * Vref - Vref / 2.0) / mVperAmp; 
    return DCCurrentValue;
}


//---------------------------------------------------------------------------------------------------------------------------------------------------------------

void loop(){
 
 
     //communication avec antennes 
  String RetourRaspPi, DemandeRaspPi; //Déclaration de 2 chaines contenant les messages de reception et de retour
  RetourRaspPi=String();//Initialisation avec rien
  DemandeRaspPi=o;
  Serial1.println(DemandeRaspPi);
  ms=0;
  while (Serial1.available() < 1)// tant que on recois rien

      {
        delay(1);
        ms=ms+1;
        if (ms>1000)

          {
               break;        
          }
      }

  if(Serial1.available() < 1)//si le nombre de caractere recu est inferieur ou egal a 0

      {    

        FlexiTimer2::start();// start du compteur d'erreur d'envoie
        Serial.println("echec");
        Serial.println(compteur);

        if (compteur>299)
           {    
            Serial.println("Pas de transmission");
            compteur=0;
            BPR=digitalRead(BP_6);
            while (BPR==0)
                {
                  BPR=digitalRead(BP_6);  //fin du programme relancer EN APPUYANT SUR BP
                }                           
           }
      }
  else 
      {
        FlexiTimer2::stop();                   // arrete du compteur d'erreur d'envoie
        compteur=0;
      }
  RetourRaspPi=Serial1.readStringUntil('#'); //Reception du renvoie de l'ecuri regarder si un message est dedans et qu'il correspond au données envoyer
  Serial.println(RetourRaspPi);
  msgtaille=RetourRaspPi.lastIndexOf("{");    //test si il y a message et la taille de la donné
  if (msgtaille == 0 )
  {
    msgtaille=RetourRaspPi.lastIndexOf("}");
    message=RetourRaspPi.substring(0+12, msgtaille-1);
    Serial.println("Vous avez recu un message de l'ecurie:"+message);
    delay(3000);
    msgtaille=-1;
  }
  if(msgtaille > 0 )                          // si on recois un message
       {
         message=RetourRaspPi.substring(msgtaille);           //séparation des données et du message
          msgtaille=message.length();                         //relever de la taille du message
          msgtaille=msgtaille-3;                              //pour enlever les 3 derrenier caratere
          message=message.substring(12,msgtaille);            // et les 12 premier pour avoir seulement le message
          Serial.println("Vous avez recu un message de l'ecurie:"+message);
          delay(3000);
       }
  else
  {
     Serial.println(""); // retour rasp^pi
  }
//--------------------------------------------------------------------------------------------------------------------------------------------------
           //capteur                          
    
        Vcapteur=(5.0/1023.0)*analogRead(capt);   //Convertion analogique numérique
        Vmesure=(Vcapteur-2.49)/0.0681 -0.25;
        Serial.print("Tension ");        
        Serial.println(Vmesure);
       
        if(digitalRead(BP_stop)){
           race_started = false;
            return ;
       }
        // Ecriture des valeurs sur le moniteur série
        float CurrentValue =  readDCCurrent(currentSensorPin) ;   
        Serial.print("Intensité ");
        Serial.println(CurrentValue); 
        
        if(digitalRead(BP_stop)){
           race_started = false;
            return ;
        }
       baterie=(Vmesure*100)/52;
       if (baterie > 99)
       {
        baterie=100; 
       }
       
//----------------------------------------------------------------------------------------------------------------------------------------------------
           //programme xbe
        Serial.println("tour(s) : ");
        Serial.println(tour);
        Serial.println(Serial3.readString());
    if(Serial3.readString() == "E" ) 
    {  

        Serial.println("if");
        tour=tour+1;
        Serial.println("tour(s) : ");
        Serial.println(tour);
        Serial3.print("a");
          
    }
//----------------------------------------------------------------------------------------------------------------------------------------------------
           //Vitesse      
      {
    float dt=0,new_vkmh=0,vkmh=0;
  if(signalDetected){
    signalDetected=false;
    if(t0>0){
      dt=(signalDetectedMillis-t0)/1000;
      new_vkmh=(pi*D*3.6)/dt;
      vkmh=(prev_vkmh+new_vkmh)/2;
      prev_vkmh=new_vkmh;
    }
    t0=signalDetectedMillis;
  }
  Serial.print("vkmh = ");
  Serial.println(vkmh);
  vkmh_flemme=vkmh;

      }
    
       char GPS;
//---------------------------------------------------------------------------------------------------------------------------------------------------
       // CODE GPS
        Serial.print(gps.location.lat(), 6);
        Serial.print(";");
        Serial.println(gps.location.lng(), 6);
        smartDelay(1000);
        if (millis() > 5000 && gps.charsProcessed() < 10)
          Serial.println(F("No GPS data received: check wiring"));
//envoie donné pour écran



if (swap==0)
{

if (CurrentValue >= 10.00)
{
  taille=5;
}
else 
{
  taille=4;
}

messagee[taille];
dtostrf(CurrentValue,taille,2,messagee);
 
swap=1;
carac='I';
}
else if (swap==1)
{

if (Vmesure >= 10.00)
{
  taille=5;
}
else 
{
  taille=4;
}

messagee[taille];
dtostrf(Vmesure,taille,2,messagee);

swap=2;
carac='U';

}
else if ( swap==2)
{
  taille=1;
    messagee[taille];
   itoa(tour,messagee,10);
   Serial.println(messagee);
   swap=3;
   carac='T';
}
else if (swap==3)
{
  taille=4;
 messagee[taille];
 dtostrf(vkmh_flemme,taille,2,messagee);
 swap=0;
 carac='V';
}



unsigned char stmp[6]={carac,messagee[0],messagee[1],messagee[2],messagee[3],messagee[4]};

    // send data:  id = 0x00, standard frame, data len = 6, stmp: data buf
    CAN.sendMsgBuf(0x70,0, 6, stmp);
    delay(10);                       // send data once per ms
    unsigned char buf[1];
    unsigned char len=0;
 if(CAN_MSGAVAIL == CAN.checkReceive()) 
 {
  CAN.readMsgBuf(&len,buf);
  char caractere=buf[0];
  caractere=caractere-'O'+79;
 Serial.println(caractere);
  if (caractere='U' && BP==0)
  {
     BP=1;
  }
  else
  {
    BP=0;
  }
  
}

         o=String(Vmesure) + ";" + String(CurrentValue) + ";" + String(gps.location.lat(), 6) + ":" + String(gps.location.lng(), 6) + ";"+String(vkmh_flemme)+";"+String(tour) +";"+ String(ms)+";"+String(BP);
         
          
digitalRead(BP_resetD);
Serial.println(distance);
if (BP_resetD == 1);
        {
        distance=0;
        }
}     

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 

  {
    while (ss.available())
    gps.encode(ss.read());
  } while (millis() - start < ms);

 }
 static void printFloat(float val, bool valid, int len, int prec)
  {
  if (!valid)
  {
    while (len-- > 1)
    Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
     Serial.print(' ');

  }
  smartDelay(0);
}

 
  //GPS
  void clearBufferArray()                     // function to clear buffer array

{
    for (int i=0; i<count;i++)

    {
        buffer[i]=NULL;
    }                      // clear all index of array with command NULL

    Serial.println(buffer[64]);
  
}
