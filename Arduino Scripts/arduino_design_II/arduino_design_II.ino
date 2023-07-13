#include <Adafruit_NeoPixel.h>
#define green 13 // Define the pin for the LED
#define red 2
#define out 18
#define in 17
int refHumid = 0;
int RH = 15;  //output is pin A1
int realHumid = 0;
int Value = 420;
float base = 0;
float base1 = 0;
float base2 = 0;
float base3 = 0;
float current = 0;
int ground_sample = 0;
int humidity_sample = false;
int input = 16;
int wood = 10;
int pipe = 11;
int vent = 6;
float delta = 0;
int ring = 3;
int numLed = 12;
int count = 0;
Adafruit_NeoPixel strip(numLed, ring, NEO_GRB + NEO_KHZ800);

void setup()
{
  strip.begin();
  Serial.begin(9600);
  pinMode(out, OUTPUT);
  pinMode(in, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(red, OUTPUT);
  pinMode(pipe, OUTPUT);
  pinMode(wood, OUTPUT);
  pinMode(vent, OUTPUT);
  
}

void material(float change)
//function takes change in voltage and determines the surface it is above
{
  if(change>=60)
  //if delta increases photo resisor is exposed to less light
  {
    Serial.println("grate");//light decrease means surface is darker, meaning the vent
    digitalWrite(vent, 1);
    digitalWrite(wood,0);
    digitalWrite(pipe,0);
  }
  else if(change<=-40)
  {
    Serial.println("pipe");// increase in light, means more reflection, meaning the pipe
    digitalWrite(vent, 0);
    digitalWrite(wood,0);
    digitalWrite(pipe,1);
  }
  else 
  {
    Serial.println("wood");// no change, still on baseline aka wook
    digitalWrite(vent, 0);
    digitalWrite(wood,1);
    digitalWrite(pipe,0);
  }
}
void light_on()
{
  for (int i = 0; i < numLed; i++) {
    strip.setPixelColor(i, strip.Color(50, 50, 50));
  }
  strip.show(); // Display the updated colors on the LEDs
}
void light_off()
{
  for (int i = 0; i < numLed; i++) 
  {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }
   strip.show(); // Display the updated colors on the LEDs
}
void loop()
{
  while (Serial.available() == 0){}
  String command = Serial.readString();
  command.trim();
  if(command == "ground"){
    current = (analogRead(input));// reads current voltage
    delta = (current - base);// gets change
    material(delta);
  }
  if(command == "humidity"){
    realHumid = analogRead(RH);
    if(realHumid >= Value){
      Serial.println("true");
      digitalWrite(green, LOW); // Turn on the LED if humidity is over the threshold
      digitalWrite(red, HIGH);
    } 
    else {
      Serial.println("false");
      digitalWrite(green, HIGH); // Turn off the LED if humidity is below the threshold
      digitalWrite(red, LOW);
    }
  }
  if(command == "mitigate"){
    if (count == 0) {     //IF the linear actuator is in position 0, move to position 1
      count++;
      digitalWrite(in, HIGH);
      delay(3000);
      digitalWrite(in, LOW);
      }
    else if (count == 1) {   //if the LA is in position 1 move to position 2
      count++;
      digitalWrite(in, HIGH);
      delay(5500);
      digitalWrite(in, LOW);
  }
    Serial.println("mitigation complete");
  }
  
  if(command == "all_clear"){
    Serial.println("green_light_on");
  }
  
  if(command == "baseline"){
    base1 = analogRead(input);//arduino takes initial voltage reading, this will be the baseline and the measurement for the wood surface
    delay(20);
    base2 = analogRead(input);
    delay(20);
    base3 = analogRead(input);
    base = (base1 + base2 + base3)/3;
    refHumid = analogRead(RH);
    Serial.println(base);
  }
  
  if(command == "light_on"){
    light_on();
    Serial.println("light_on");
  }
  
  if(command == "light_off"){
    light_off();
    Serial.println("light_off");
  }
  
  if(command == "green_on"){
    for (int i = 0; i < numLed; i++) {
    strip.setPixelColor(i, strip.Color(0, 50, 0));
      }
    strip.show();
    Serial.println("green_on");
  }
  
  if(command == "red_on"){
    for (int i = 0; i < numLed; i++) {
    strip.setPixelColor(i, strip.Color(50, 0, 0));
      }
    strip.show();
    Serial.println("red_on");
    }
  
  if(command == "reset"){
    if (count == 1) {     //reset to position 0 from position 1
      digitalWrite(out, HIGH);
      delay(3000);
      digitalWrite(out, LOW);
      count = 0;
    } 
    else if (count == 2) {      //reset to position 0 from position 2
      count = 0;
      digitalWrite(out, HIGH);
      delay(8500);
      digitalWrite(out, LOW);
    } 
    else {                    // //reset to position 0 from position 0
      count = 0;
    }
  }

  delay(100);
}
