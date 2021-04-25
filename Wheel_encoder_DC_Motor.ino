/*
 PID Algorithm it automatically applies an accurate and responsive correction to a control function
 Is a control loop mechanism employing feedback
*/
//PID constants

double Kp;      // is the proportional gain, a tuning parameter
double Ki;      //is the integral gain, a tuning parameter
double Kd;      //is the derivative gain, a tuning parameter,
                //Tuning a control loop is the adjustment of its control parameters ,
                //to the optimum values for the desired control response
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output;
double setPoint;        //Target Value;
double cumError, rateError;
//----------------------------------------------------------------------------------------------
/*
 * Encoder_A and Encoder_B pins are used to read the encoder data from the microcontroller
 * data comes so fast so these two pins have to interrupt enabled pins
 */
#define ENCODER_A 2
#define ENCODER_B 3
/* The next variables are for Motor Driver
 * They are used to control motor speed and direction
 * We control the direction by the H-bridge
 * An H-bridge is an electronic circuit that switches the polarity of a voltage applied to a load. 
 * These circuits are often used in robotics and other applications 
 * to allow DC motors to run forwards or backwards
*/
#define PWM 5             //Motor Driver PWM input
#define IN2 6             //Motor Driver input
#define IN1 7             //Motor Driver input
int pos;                  //to store the position of the motor shaft

/*
 * This Fucntion is used to set the motor direction and speed
 * it takes Parameters (rotation direction , PWM Speed , PWM Pin ,in1 pin , in2 Pin)
 */

void setMotor(int dir , int pwmVal , int pwm , int in1 ,int in2)
{
  analogWrite(pwm,pwmVal);             //Setting the Speed
  if(dir==1)
  {
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if (dir == -1)
  {
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    }
    else
    {
      digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
      }
  }

void setup() {
  setPoint = 0; //set point at zero degrees
  Serial.begin(9600);
  pinMode(ENCODER_A, INPUT); // ENCODER_A as Input
  pinMode(ENCODER_B, INPUT); // ENCODER_B as Input
  pinMode(IN2, OUTPUT); // MOTOR_CW as Output
  pinMode(IN1, OUTPUT); // MOTOR_CW as Output
  /* attach an interrupt to pin ENCODER_A of the Arduino, 
   *  and when the pulse is in the RISING edge called the function -----.
*/
  attachInterrupt(digitalPinToInterrupt(ENCODER_A),readEncoder,RISING);
}

void loop() {
  setMotor(1 , 25,PWM,IN1,IN2);
  delay(200);
  Serial.println(pos);

}

double computePID(double inp)
{
  currentTime = millis();                              //get the current time
  elapsedTime = (double) (currentTime-previousTime);   //Calculate ElapsedTime
  error = setPoint - inp;                              //Calculate the error by get the difference between target and measured value
  cumError += error * elapsedTime;                     // compute integral
  rateError = (error - lastError)/elapsedTime;         // compute derivative
  double out = Kp*error + Ki*cumError + Kd*rateError;
  lastError = error;                                   //Remember Current error
  previousTime = currentTime;                          //Remember Current time
  return out;                                          //This signal tells us the direction and speed to turn the motor
}
/*
* The signal that comes from PID Algorithm we need to convert it to speed and direction
*/
void convertSignal(float outSignal)
{
  float pwr =  fabs(outSignal);
  if(pwr > 255)
    pwr =255;
  int dir = 1;
  if(outSignal < 0)
    dir = -1;
    setMotor(dir , pwr ,PWM,IN1,IN2);
  }
  //Get the position of the motor
void readEncoder()
{
  if(digitalRead(ENCODER_B)==HIGH)
  {
    pos++;
    }
    else
    {
      pos--;
    }
}
