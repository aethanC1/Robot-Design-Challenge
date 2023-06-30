float current = 0;
float base = 0;
int input = 15;
int wood = 10;
int pipe = 11;
int vent = 6;
float lastDelta = 0;
float delta = 0;
void setup()
{
  Serial.begin(9600);
  base = analogRead(input);//arduino takes initial voltage reading, this will be the baseline and the measurement for the wood surface
  pinMode(pipe, OUTPUT);
  pinMode(wood, OUTPUT);
  pinMode(vent, OUTPUT);
}

void material(float change)
//function takes change in voltage and determines the surface it is above
{
  if(change>=100)
  //if delta increases photo resisor is exposed to less light
  {
    Serial.println("on vent");//light decrease means surface is darker, meaning the vent
    digitalWrite(vent, 1);
    digitalWrite(wood,0);
    digitalWrite(pipe,0);
  }
  else if(change<=-50)
  {
    Serial.println("on pipe");// increase in light, means more reflection, meaning the pipe
    digitalWrite(vent, 0);
    digitalWrite(wood,0);
    digitalWrite(pipe,1);
  }
  else 
  {
    Serial.println("on wood");// no change, still on baseline aka wook
    digitalWrite(vent, 0);
    digitalWrite(wood,1);
    digitalWrite(pipe,0);
  }
}
void loop()
{
  lastDelta = delta;
  current = (analogRead(input));// reads current voltage
  delta = (current - base);// gets change
  Serial.println(delta);
  material(delta);//checks surface
  delay(150);
}
