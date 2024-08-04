const int trigPin1 =  2 ;  // ball 
const int echoPin1 = 3 ;   // ball 

const int trigPin2 =  10 ;  // hand 
const int echoPin2 = 11 ;   // hand 

 // gains
double Kp=4.5;
double Ki=1.5;
double Kd=1.17;
  
float  duration1, duration2, distance1, distance2  ;


 int motorpin1 =  6 ;  // motor 
 int motorpin2 =  7 ;  // motor 
 int enA = 9 ;   // motor 

double dt, integral, previous, output, lasttime = 0, lowerpwm=180, upperpwm=255, rangecm=40 ;
double rangepwm= 70;

void setup() {


  // for ultrasonic
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  //for motor
  pinMode(motorpin1, OUTPUT);
  pinMode(motorpin2, OUTPUT);
  pinMode(enA, OUTPUT);

  //setting direction for the motor
  digitalWrite(motorpin1,LOW);
  digitalWrite(motorpin2,HIGH);
  
  Serial.begin(9600);
}

void loop() {

  //triggering pins for ultrasonic
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

 duration1 = pulseIn(echoPin1, HIGH);
  
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  // duration and distance calc

  
  duration2 = pulseIn(echoPin2, HIGH);

  distance1 = (duration1*.0343)/2; // ball
  distance2 = (duration2*.0343)/2; // hand 
    
  // calculating change in time 
  double now = millis();
  dt = (now - lasttime)/1000.00 ;
  lasttime = now ;
if(distance2>2 && distance2<41){
  double refrence = 50 - distance1;
  double errorincm = refrence - distance2;
  double currentpwm= map(analogRead(enA), 0, 255, 0, 1023);
  double desiredpwm = (errorincm/rangecm*100*rangepwm)+lowerpwm;
  double errorinpwm =currentpwm- desiredpwm;
  output = pid(errorinpwm);
  analogWrite(enA, output);
}
else{analogWrite(enA,207);}
  //printing distances 
  Serial.print("Distance: ");
  Serial.println(distance1);
  Serial.print("Distance2: ");
  Serial.println(distance2);
  Serial.print("pwm: ");
  Serial.println(output);
 
}


double pid(double error){

  double proportional = error;
  integral += error * dt;
  double derivative = (error-previous)/dt;
  previous = error;
  if(distance2<14){
 double kpa= 3.8;
Kp=kpa;
double kia= 1.5;
Ki=kia;
 double kda= 1.16;
Kd=kda;

  }
  else if(distance2<14 && distance2>36){
double kpb= 4.3;
Kp=kpb;
    double kib= 1.2;
Ki=kib;
 double kdb= 2;
Kd=kdb;

  }
  else{
    double kpc= 4.69;
Kp=kpc;
    double kic= 1.66;
Ki=kic;
    double kdc= 2.1;
Kd=kdc;

  }
  output = (Kp*proportional) + (Ki*integral) + (Kd*derivative);

  if (output>upperpwm){
  output=255;
  }

  else if (output<lowerpwm){
  output=180;
  }

  return output;

 }