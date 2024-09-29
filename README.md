As you can see, our discussion is divided into two main topics: Control and PID. Let's begin by explaining the first concept **Control**.
What do we mean by **Control** ? and what is **Control systems** ?
#### What is Control systems ? 
We can simplify the term into the following figure :
![[Pasted image 20240927200133.png]]
A control system aims to determine and generate the appropriate input to achieve the desired output !
But can we actually do that with  such a simple system (An open loop system) ? 
#### Open-Loop system 
 
The answer to the previous question is NO. As we all know, we don't live in a perfect world, and it's extremely difficult (if not impossible) to achieve the desired output in an open-loop system. For example, imagine you're driving a car and want to maintain a speed of 130 km/h. Various factors, such as driving uphill or on a rocky road, will make it impossible to keep your speed steady at 130 km/h. This is where the closed-loop system comes into play!
#### Closed-Loop system 
In the following picture, we can see the closed loop architecture :
![[Pasted image 20240927201443.png]]
As you can see in the closed-loop architecture, we have a feedback path, which in our case consists of sensors. These sensors continuously measure the system's performance, allowing us to calculate the error—essentially, the difference between the actual system output and the desired command from the operator. Our goal is to apply a mathematical model that will drive this error to zero, ensuring that the controlled variable (output) matches the commanded variable (input or desired value).
### PID Controller ?
Remember when we talked about the mathematical model that drives the error to zero? That model is called a controller, and the PID controller is just one example among many types of controllers.

#### Example : Human Position control 
Imagine you're standing at the start of a football field, and your goal is to walk 50 meters to reach the center. In this case, your desired position (the input) is 50 meters, you are the plant, and your eyes and senses serve as the feedback mechanism.  
At t=0, the error is 50 meters, meaning you're far from your goal. Your brain, acting as the controller, processes this information and tells you to walk faster to reduce the error!
![[Pasted image 20240927203251.png]]
#### Proportional controller : (P)
Here, we're setting the controller with a simple proportional gain (P) value of 0.5.  
At t=0, when the error is at its maximum, our brain (the controller) tells us to move at a speed of 25 meters per minute. As we progress and the error decreases, our speed gradually slows down. This is because the controller continuously adjusts the output in response to the shrinking error.  
For a better understanding, take a look at the following graph :
![[Pasted image 20240927203904.png]]
Enough theory—let's dive into coding! We'll start with a simple Proportional Controller in C/C++ that we can later implement on our microcontroller:

```cpp
float Pcontroller(float setpoint, float measurement, float kp) {
    float current_error;
    float pcontOutput;
    
    // Calculate current error
    current_error = setpoint - measurement;
    
    // Proportional controller output
    pcontOutput = kp * current_error;
    
    // Clamp output between 0 and 255 for PWM range
    if(pcontOutput < 0)
        pcontOutput = 0;
    else if(pcontOutput > 255)
        pcontOutput = 255;

    return pcontOutput; // Return velocity command for motors (PWM value)
}
```
But keep in mind, we need to carefully choose the right Kp value to maintain system stability.  
Here’s how the system's response changes based on different Kp values:
![[Pasted image 20240927223738.png]]

So, it seems that a simple proportional gain works! Yes, but only in theory :)  
In a real-life example, like controlling a monowheel robot, as we approach the desired position and the error gets smaller, we might end up sending an insufficient command to the motor (the plant). This happens because the motor won't receive enough power to overcome the torque and friction, eventually causing it to stop before reaching the target. This issue is known as "Steady-State Error" or "Static Error."

To eliminate this static error, a **proportional-integral (PI) controller** is often used. The integral action accumulates the error over time, allowing the controller to generate additional control effort, even when the proportional action alone isn't sufficient.

#### Proportional-Integral controller : (PI)
The integral component we just added utilizes past information to adjust our system toward the desired output!
![[Pasted image 20240927212046.png]]
The integrator works by accumulating the error over time. If we have a constant error, the integrator will generate a ramp that is added to the output velocity command, helping our system reach the desired position. With the integrator in place, achieving zero error is theoretically possible, but this is under ideal conditions.

In practice, the system may overshoot slightly, resulting in a negative contribution to the integrator. This, in turn, decreases the velocity command, leading to error once again, creating a cycle of oscillation until the system stabilizes. These oscillations can be detrimental, indicating partial instability in the system.

Therefore, it is crucial to determine the optimal integral gain (Ki) that provides the best performance.

First of all , we have to know a very important aspect in Arduino which is the *millis()* function .
### How `millis()` Works:

- `millis()` keeps track of time in milliseconds (1 second = 1000 milliseconds) since the program started running.
- The function returns an `unsigned long` type, meaning it can store very large values (up to about 49 days before it rolls over to 0).
- Unlike `delay()`, `millis()` is non-blocking, which means it allows other parts of your code to run while it is counting time.

### Why Use `millis()`:

- To create non-blocking delays (without freezing the rest of the program).
- To schedule tasks at regular intervals, like reading sensor values or blinking an LED periodically.

### Example: Blinking an LED with `millis()`

Here’s an example of using `millis()` to blink an LED every 1 second:

```cpp
const int ledPin = 13; // Pin for the LED
unsigned long previousMillis = 0; // Stores the last time the LED was updated
const long interval = 1000; // Interval of 1 second (1000 milliseconds)

void setup() {
  pinMode(ledPin, OUTPUT); // Set the LED pin as an output
}

void loop() {
  unsigned long currentMillis = millis(); // Get the current time

  // Check if the interval has passed
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Update the time

    // Toggle the LED state
    int ledState = digitalRead(ledPin); // Read the current state of the LED
    digitalWrite(ledPin, !ledState); // Set the LED to the opposite state (on/off)
  }
}
```

Now we have all the necessary components to code our PI Controller. We will utilize the concepts we've discussed earlier.  
As an example, we will develop a PI controller for voltage control. The process works as follows: we will output a PWM signal through one of the pins, which will then pass through an RC filter (acting as the integrator) to smooth the signal. The smoothed output will return to the MCU through an analog pin, allowing us to regulate the voltage effectively!
![[Pasted image 20240929122911.png]]
![[Pasted image 20240929122928.png]]

Now let's code it : 
```cpp
/*Wiring PINS*/
#define PIN_OUTPUT  3
#define PIN_INPUT   A0

/*Regulator PIN*/
#define Kp          0
#define Ki          0

unsigned long previousTime,now;
double dt=0;
double output,integral,previousError=0;
double setpoint = 100 ;

void setup()
pinMode(PIN_OUTPUT,OUTPUT);
analogWrite(PIN_OUTPUT,0);
previousTime = 0;
}

  
void loop(){
    now = millis();
    dt = (now-previousTime)/1000;
    previousTime=now;
    double feedback =map(analogRead(PIN_INPUT),0,1024,0,255);
    double error = setpoint - feedback;
    output= pid(error):
	analogWrite(PIN_OUTPUT,output);
    delay(50);
}

  
double pid(double error){

    double prop= error;
    integral += error * dt ;
    previousError=error;
    output = (Kp*prop) + (Ki*integral);

}
```

We achieved our goal, but not without issues: the signal overshoots when reaching the setpoint, leading to oscillations. These oscillations can be detrimental to our system, so it’s essential to eliminate them. This is where the derivative action comes into play!
![[Pasted image 20240929124156.png]]
#### Proportional-Integral-Derivative controller : (PID)
The derivative action in a PID controller plays a crucial role in reducing oscillations and overshooting as the system approaches the setpoint. It achieves this by responding to the rate of change of the error, predicting future behavior, and applying corrective action accordingly. Essentially, it 'dampens' the system's response, slowing down rapid changes and enabling smoother adjustments. This prevents the system from reacting too aggressively, enhancing its stability and responsiveness to disturbances. By incorporating the derivative action, the controller can anticipate and minimize overshooting before it becomes an issue.

Now, let's dive into the coding part:
```cpp
/*Wiring PINS*/
#define PIN_OUTPUT  3
#define PIN_INPUT   A0

/*Regulator PIN*/
#define Kp          0
#define Ki          0
#define Kd          0

unsigned long previousTime,now;
double dt=0;
double output,integral,previousError=0;
double setpoint = 100 ;

void setup()
pinMode(PIN_OUTPUT,OUTPUT);
analogWrite(PIN_OUTPUT,0);
previousTime = 0;
}

  
void loop(){
    now = millis();
    dt = (now-previousTime)/1000;
    previousTime=now;
    double feedback =map(analogRead(PIN_INPUT),0,1024,0,255);
    double error = setpoint - feedback;
    output= pid(error):
	analogWrite(PIN_OUTPUT,output);
    delay(50);
}


double pid(double error){

    double prop= error;
    integral += error * dt ;
    double derivative= (error-previousError)/dt ; 
    previousError=error;
    output = (Kp*prop) + (Ki*integral) + (Kd*derivative);

}
```


---

**WARNING:**

- The derivative action should only be applied to slow systems!
- In high-frequency scenarios, the derivative action can have adverse effects, potentially causing instability in the system. Therefore, it is essential to apply a low-pass filter before implementing the derivative action!

You can plot any variable through *serial.print()* and observe your system feedback to tune it ! 