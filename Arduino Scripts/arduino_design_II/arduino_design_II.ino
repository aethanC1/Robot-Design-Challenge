
float base = 0;
float base1 = 0;
float base2 = 0;
float base3 = 0;
float current = 0;
int ground_sample = 0;
int humidity_sample = false;
int input = 15;
int wood = 10;
int pipe = 11;
int vent = 6;
float delta = 0;

void setup()
{
  Serial.begin(9600);
  base1 = analogRead(input);//arduino takes initial voltage reading, this will be the baseline and the measurement for the wood surface
  delay(20);
  base2 = analogRead(input);
  delay(20);
  base3 = analogRead(input);
  base = (base1 + base2 + base3)/3;
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
    Serial.println("grate");//light decrease means surface is darker, meaning the vent
    digitalWrite(vent, 1);
    digitalWrite(wood,0);
    digitalWrite(pipe,0);
  }
  else if(change<=-50)
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
    if (humidity_sample == true) {Serial.println("true");}
    else {Serial.println("false");}
  }
  if(command == "mitigate"){
    Serial.println("mitigation complete");
  }

  delay(100);
}
