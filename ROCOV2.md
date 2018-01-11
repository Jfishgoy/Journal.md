# **Journal.MD** 
============

#**lab 1: Github and hack a robot**
===========
 

## Markdown syntax
markdown is described as a lightweight markup language with plain text formating syntax. it has a number of uses, mainly utilised for its malleability, it can be used in HTML and readme formats to name a few.

To highlight a few rules, see below.
creating headers using #, using more #s to reduce the header size.

#Header
###Header

underlines can be created using ==== or -------

line
====

**Emphasis**
====
emphasis can be implemented by using either asterisks* or underscores_
additional emphasis can be added using another of the characters

*light* *emphasis*
**bold** **emphasis**

**strikethroughs**
====
strikethroughs can be used using two tildes(~) either side of the word/phrase

~~struck~~

**lists**
====
lists can be used using full stops and asterisks

1. item one
* indent item one
2. another item

**links**
===
links can be used in two different ways, with or without a title

[without title](https://www.google.com)
[with title](https://www.google.com "google homepage")


**highlighting**
===
highlghting can be used by three back ticks '''
'''code'''

**quote**
===
quotes can be used using a ">" symbol
>this is a quote

raw HTMl can be used in markdown too

**task 2 the terminal**
====
$ ls
ls lists all files on the computer and sperates them into their respective locations

$ cd /tmp
changes the current directory to /tmp

$ cd $HOME
changes current directory to HOME

$ mkdir
returns with an error because no directory to make is specified, adding a directory name at the end of this command would create a directory.

$ echo "hello" > hello.md
creates a file called hello.md

$ cat hello.md
prints hello to the terminal

$ cp hello.md hello-again.md

$ mv hello-again.md hello-hello.md
moves contents of hello-again.md to hello-hello.md

$ rm hello.md
removes hello.md

$ rm -rf
removes directory rf

$ cat /proc/cpuinfo
prints the specs of the current computer in the terminal

#**lab 2: build a DC motor**
=========

our second task was to build a functional dc motor from fundamental parts given to us. The motor was to be built upon in the coming practicals.


DC motors are incredibly common as they were the first type of motor that was widely used worldwide. the principle of a working DC motor is *whenever a current
carrying conductor is placed in a magnetic field, it experiences a mechanical force*. the direction of this induced force is dictated by *Fleming's left hand rule*:

>F = BIL

they operate by turning electrical energy into mechanical energy. in a typical DC motor, the motor armature, or **rotor**, is suspended between peramanent magnets,
the magnets being stationary, These magnets are called the **stators**. The suspended armature is wired like an electromagnet with coils of wire surrounding the 
armature core. In order for the motor to rotate and function, power is supplied to the armature coils, generatiing a magnetic field throughout the armature.
this in turn will attract and repel against the stators, generating torque. this torque is unidirectional due to the armatures segmented coils.
This will, however, only push the rotor through a turn of 180degrees, as the stator will keep 
the rotor in place at this point because of the static magnetic field. This is why the electromagnet in the armature switches polarity as it rotates. It does this by periodically making contact
with two electrodes, that can be either attached to the armature or remain stationary. these electrodes allow the polarity of the armature to change by reversing the 
direction of current flowing through the coils.

![Diagram of Brushed DC motor](https://www.electrical4u.com/electrical/wp-content/uploads/2013/03/dc-motor-parts.jpg)
this diagram shows a brushed DC motor, the same type as we were instructed to build. It, unlike our motor, shows a singled wire armature. 
They can be found in all manner of appliances in all sizes, from small toys to large engines and elevators.


###Construction
--------
to begin with, we wound around **10M** of enameled copper wire around the armature core we were given (a bottle cork) length-ways. this equated to around 100
turns, the ends of the wire were frayed to allow electricity to pass into the wire. these ends of the wire were then attached via soldering to our commutators (copper tape) to create our rotor.
additional measures were taken to ensure the rotors stabililty, such as tape aroubnd the cork.
<image>

then, we construced to motors base to hold the rotor and the stators. To do this, we were given thick metal paperclips and a small plank of wood. after bending the paperclips into position, 
they were secured to the plank using a wide washer and screws. The stator magnets were fixed in position and a "cradle" was construced to suspend the rotor in position whilst still allowing
movement.
<image>

and below is the finished product of the construction, and [here](https://www.youtube.com/watch?v=kQmZnGIjSNc&feature=youtu.be "Demonstration Video") is a link to a video
demonstrating its operation.

At this point the next stage of our task was improving the DC motor design. we were given a few avenues to pursue in this notion: 

>• Can you build an elegant way of holding the brushed to apply current to the commentator?
>• Can you improve the way the armature rotates by using better bearings?
>• How would increasing the number of turns of wire affect motor characteristics?
>• Consider this in terms of speed, torque and current consumption

to begin with, we rearranged the armature. To improve on it, we gave the armature a second coil. Doing this would help the motor increase it's resolution and
help with the commutation process, also increasing the resolution. both of these coils were wound 100 times and followed the steps taken previously. the copper plate commutators were redone as there 
were now 4 of them. They, like last time, were made sure to be as smooth as possible and cover as much of the circumfrence of the rotor as possible without touching.


#**Lab 3: Incremental Encoder**
===============

the task given to us was to build and attach an incremental encoder to our pre-built DC motor, 
it was to be programmed and managed by an arduino program. incentive was given to improve the resolution of the encoder and to understand its operation.



![Incremental encoder operation](http://hades.mech.northwestern.edu/images/2/22/Encoder_diagram.png)
An incremental encoder is a device used to measure the rotational speed of a disc, there are many methods to achieve this end.
 the method of operation is a disc is attatched to a motor shaft. The disc aides with calculating the rotational speed. 
In some cases, a hall effect sensor is used to detect magnets located on the disc, from which the speed, 
direction and location of the motor shaft can be determined. In other cases, an infrared LED and infrared receiver are positioned on either side of the rotational disc, 
except in this case, the rotational disc has some sort of opening, or hole, cut into it. This means the infrared beam will be interrupted until it passes through the hole in the circle, 
making a connection with a pulse. From this pulse the speed of the motor can be determined. If a second LED transmitter and receiver are added to the motor shaft, 
the relative pulses from both receivers can be used to determine the direction of the motor as well as the speed. 
Incremental encoders with the above operations are normally used in permanent magnet brushless motors, to track the rotors position,
 as well as in computer numerical control (CNC) systems and industrial equipment.



For this lab session, we were given the task of building a incremental encoder with an LED transmitter and receiver as its method of operation. 
the disc would be attached to the DC motor we built in the previous laboratory session, and then an industrial motor to compare functionality.

<Circuit Diagram>
Above is the circuit diagram for the infrared transmitter and receiver for the encoder, 
we used the schematic to help us build the circuit. not much trouble was encountered building the circuit, 
but some re-wiring was necessary. in the end, the circuit was fully functional and we were ready to move on to the next stage of the building process, 
attaching the encoder disc.



The rotational disc was made of a thin sheet of cardboard, fashioned into a circle. A small arc was cut into the circle, not deep enough
to reach the centre of the disc though, as this might reduce the structual integrity of the disc as it rotates quickly. 
There was little trouble arranging the disc on the end of the motor shaft.

with the encoder fully assembled, we tested its functionailty. [Here](https://www.youtube.com/watch?v=B1t7ziU7dFo "Demonstration") is a link to the demonstration of
the encoder, with a few seconds showing the oscilloscope readings too. 

In the video, you can see some noise on the rising and falling edges of the square wave, we believe this is due to the size of the arc cut out and the method of its cutting,
using scissors may not have given the cleanest cut on cardboard of the discs thickness. The noise may also be due to the the Transmitter and receivers' proximity to the circuit board.
 In addition to that, we found that the speed of the motor had no effect on the width of the square wave, it simply increased or decreased in frequency with speed. 
Changing the size of the cut would change the width of the square wave, proportional to the amount of the disc missing.

below is the arduino code that was modified from the sample code. It was modified to calculate angular velocity by calculating the number of pulses per cycle.

>const byte ledPin = 13;
>const byte interruptPin = 2;
>volatile byte state = LOW;
>volatile int PCount;
>
>void PCounter(){
> Pcount++; }
>
>void setup() {
> Serial.begin(9600);
> pinMode(ledPin, OUTPUT);
> pinMode(interruptPin, INPUT);
> // configure the interrupt call-back: blink is called everytime the pin
> // goes from low to high.
> attachInterrupt(digitalPinToInterrupt(interruptPin), blink, RISING); }
>
>void loop() {
> Pcount = 0; //reset to 0
> delay(20);
> digitalWrite(ledPin, state);
> int result = Pcount; //holds value
> Serial.print(F("Pulse Count = "));
> Serial.println(result); //output to serial }
>
>void blink() {
>state = !state; }

In conclusion, improvements could be made to this system. To begin, a rig could be made for the motor to improve stability,
as well as a cleaner square wave. I would also make changes to the Transmitter/ Reciever circuit to improve its performance.
I would create more distance between the circuit and its effectors, to reduce noise from the circuit. In addition, changing
the design of the cardboard circle could produce more valid data. By dividing the discs circumfrence into 4 equal arcs, 
an easier to read square wave would be produced. this one would would have two pulses equidistant in a complete revolution,
as opposed to a single pulse during the revolution. 

#**Lab 4: Controlling The Motor**
=============

The fourth lab task was to use our DC motor in conjunction with arduino code to better control the motor. Arduino is an 
Open-source hard/ software company and user community that specializes in single-board microcontrollers and microcontroller kits.

The first part of the task was to simply power a motor using the arduino interface. We used pins 3, 8, 9, 11, 12 and 13 and switched them 
all high sending a constant flow of electricity through the pins, thereby powering the motor. Pins 12 and 13 in this case can be set
to either HIGH or LOW, as they determine motor direction and do not power the motor, with HIGH indicating forwards, and LOW backwards.
pins 3 and 11 were also used for a function other than powering motors, they are used to detemrine the speed of the motor via 
Pulse Width Modulation (PWM), in this instance they were set to the maximum value of 255.

PWM is a process by which a variable amount of power can be supplied to a component by altering the duty cylce of said component, represented by a percentage.
A common example of PWM at work is dimming LEDs, where the duty cycle percentage applied to them is showcased to the user via their brightness, with a smaller
duty cylce resulting in a dimmer LED. The duty cycle can be further explained by comaparing timelines of different duty cylces.

![comparison of Duty Cycles](https://cdn.sparkfun.com/assets/f/9/c/8/a/512e869bce395fbc64000002.JPG)
In the image above, we can see how the percentage of the cuty cylce indicates the amount of time the component will have power over a time period.
This is an effective way to alter performance of components that have a minimum voltage to operate.

the motors on the arduino board operate through an H-bridge, a circuit that allows the choosing of the motor direction via code.
![H-Bridge Circuit](https://electrosome.com/wp-content/uploads/2013/05/Basic-H-Bridge-Circuit.jpg)

Above is a simple H-bridge circuit, as we can see, the motor will only run correctly when S1 and S4 are turned on, or when S2 and S3 are.
Any other combination will cause the motor to fault. But by using these pairs of switches, we as the user can decide the direction of the motor by
choosing from which direction current flows into the motor.

Below is the code we used in order to make the motor follow some basic operations

>int delaylength = 20;
>int speed = 255;
>void setup() {
>pinMode(12, OUTPUT); //HIGH = forwards and LOW = backwards
>pinMode(13, OUTPUT); //HIGH = forwards and LOW = backwards
>pinMode(9, OUTPUT);
>pinMode(8, OUTPUT);}
>
>void loop() {
>digitalWrite(9, LOW);
>digitalWrite(8, HIGH);
>digitalWrite(12, HIGH);
>analogWrite(3, speed);
>delay(delaylength);
>
>digitalWrite(9, HIGH);
>digitalWrite(8, LOW);
>digitalWrite(13, LOW);
>analogWrite(11, speed);
>delay(delaylength);
>
>digitalWrite(9, LOW);
>digitalWrite(8, HIGH);
>digitalWrite(12, LOW);
>analogWrite(3, speed);
>delay(delaylength);
>
>digitalWrite(9, HIGH);
>digitalWrite(8, LOW);
>digitalWrite(13, HIGH);
>analogWrite(11, speed);
>delay(delaylength);}

[Here](https://www.youtube.com/watch?v=N6enz24aQUU&feature=youtu.be "Demonstration") is a link to the
demonstration of said motor.

TBC

#**Lab 6: Stepper Motors**
===================

Stepper Motors are DC motors that move in discrete steps. Like DC motors, they have numerous pairs of coils, 
these are instead called *phases*. The core armature of stepper motors is a permanent magnet. 


![Stepper motor Operation](https://www.pc-control.co.uk/images/step-motor1.gif)
In this case the stepper motor has 4 coils surrounding it.
The operation works as follows:
-all coils start with no power, leaving the rotor in an idle position.
-the first coil in the intended direction is supplied with power, whilst leaving the others powerless.
-after a designated time, the second coil is supplied with power and the first coil has its power removed.
-this process repeats with the incrementing coils being turned on and the previous coild turned off.
This operation is known as full stepping, because 1 phase is used at a time.

there are 3 other methods of controlling stepper motors half-stepping, micro-stepping and double-stepping.


half-stepping: double the resolution of operation by turning on two electromagnets at a time. In a 4 coil stepper motor the 
order of electromagnets with power would go 1, 12, 2, 23, 3, 31, 1
![Half-stepping](http://www.imagesco.com/images/picstepper/fig02.gif)

Micro-stepping: utilise a sin or cosine wave or incremental increases in the coils sequentially in order to increase the 
resolution of the stepper motor revolution to significantly higher levels.
![Micro-Stepping](https://hackadaycom.files.wordpress.com/2016/08/microstepping_exlained-031.png)
above we can see Coil A recieve incrementally less power, proportional to the power Coil B recieves.

Double-Stepping: similar to half-stepping in that two coils are powered at once, but misses the intermediary step when a 
single coil is powered.
![Comapring Full step with Double step](https://www.rs-online.com/designspark/rel-assets/ds-assets/uploads/images/559b8af46dec4ce5b7d823a50ab544b9Picture1.jpg)

Below is the code we used to achieve full step operation. [Here](https://www.youtube.com/watch?v=Tzk54A5l6F8 "Demonstration") is a link to the demonstration.
>void setup() {
> // put your setup code here, to run once:
>pinMode(12, OUTPUT);
>pinMode(9, OUTPUT);
>pinMode(8, OUTPUT);
>pinMode(13, OUTPUT);
>}
>void Afwd() { //Motor A Forward
> digitalWrite(12, HIGH);
> digitalWrite(9, LOW);
>}
>void Astop() { //Motor A Stop
> digitalWrite(12, LOW);
> digitalWrite(9, HIGH);
>}
>void Aback(){ //Motor A Back
> digitalWrite(12, LOW);
> digitalWrite(9, LOW);
>}
>void Bfwd(){ //Motor B Forward
> digitalWrite(13, HIGH);
> digitalWrite(8, LOW);}
>void Bstop(){ //Motor B Stop
> digitalWrite(13, LOW);
> digitalWrite(8, HIGH);
>}
>void Bback(){ //Motor B Back
> digitalWrite(13, LOW);
> digitalWrite(8, LOW);
>}
>void loop() {
> analogWrite (3, 255);
> analogWrite (11, 255);
> Afwd();
> Bfwd();
> delay(10);
> Aback();
> Bfwd();
> delay(10);
> Aback();
> Bback();
> delay(10);
> Afwd();
> Bback();
> delay(10); }

Below is the code we used to achieve double step operation. [Here](https://www.youtube.com/watch?v=fhwTeYK99GA "Demonstration") is a link to the demonstration.
>void setup() {
>void loop() {
> analogWrite (3, 255);
> analogWrite (11, 255);
> Afwd();
> Bstop();
> delay(10);
> Afwd();
> Bfwd();
> delay(10);
> Astop();
> Bfwd();
> delay(10);
> Aback();
> Bfwd();
> delay(10);
> Aback();
> Bstop();
> delay(10);
> Aback();
> Bback();
> delay(10);
> Astop();
> Bback();
> delay(10);
> Afwd();
> Bback();
> delay(10);
>}



Below is the code we used to achieve half step operation. [Here](https://www.youtube.com/watch?v=i3rMahLOW1I "Demonstration") is a link to the demonstration.

>void Bfwdhalf(){
> analogWrite (11, 127);
> digitalWrite(13, HIGH);
> digitalWrite(8, LOW);
>}
>void Bbackhalf(){
> analogWrite (11, 127);
> digitalWrite(13, LOW);
> digitalWrite(8, LOW);
>}
>void Afwdhalf() {
> analogWrite (3, 127);
> digitalWrite(12, HIGH);
> digitalWrite(9, LOW);
>}
>void Abackhalf(){
> analogWrite (3, 127);
> digitalWrite(12, LOW);
> digitalWrite(9, LOW);
>}
>void loop() {
> Afwd(); //1
> Bfwd();
> delay(10);
> Afwd(); //2
> Bfwdhalf();
> delay(10);
> Afwd(); //3
> Bstop();
> delay(10);
> Afwd(); //4
> Bbackhalf();
> delay(10);
> Afwd(); //5
> Bback();
> delay(10);
> Afwdhalf(); //6
> Bback();
> delay(10);
> Astop(); //7
> Bback();
> delay(10);
> Abackhalf();//8
> Bback();
> delay(10);
> Aback(); //9
> Bback();
> delay(10);
> Aback(); //10
> Bbackhalf();
> delay(10);
> Aback(); //11
> Bstop();
> delay(10);
> Aback(); //12
> Bfwdhalf();
> delay(10);
> Aback(); //13
> Bfwd();
> delay(10);
> Abackhalf();//14
> Bfwd();
> delay(10);
> Astop(); //15
> Bfwd();
> delay(10);
> Afwdhalf(); //16
> Bfwd();
> delay(10);
>}

This initial code was deemed by us too messy to be the final version, thus we created a variable called "check",
where A/B HL stand for the poles of the motor (A or B) going High or Low.

>void setup() {
> // put your setup code here, to run once:
>pinMode(12, OUTPUT);
>pinMode(9, OUTPUT);
>pinMode(8, OUTPUT);
>pinMode(13, OUTPUT);
>}
>int check (int Aspeed, boolean AHL1, boolean AHL2, int Bspeed, boolean BHL1, boolean BHL2){
> analogWrite(3, Aspeed);
> digitalWrite(12, AHL1);
> digitalWrite(9, AHL2);
> analogWrite(11, Bspeed);
> digitalWrite(13, BHL1);
> digitalWrite(8, BHL2);
> delay(10);
>}
>void loop() {
>check(255, HIGH, LOW, 255, HIGH, LOW);//fwdfwd
>check(255, HIGH, LOW, 127, HIGH, LOW);//fwdfwdhalf
>check(255, HIGH, LOW, 0, LOW, HIGH);//fwdstop
>check(255, HIGH, LOW, 127, LOW, LOW);//fwdbackhalf
>check(255, HIGH, LOW, 255, LOW, LOW);//fwdback
>check(127, HIGH, LOW, 255, LOW, LOW);//fwdhalfback
>check(0, LOW, HIGH, 255, LOW, LOW);//stopback
>check(127, LOW, LOW, 255, LOW, LOW);//backhalfback
>check(255, LOW, LOW, 255, LOW, LOW);//backback
>check(255, LOW, LOW, 127, LOW, LOW);//backbackhalf
>check(255, LOW, LOW, 0, LOW, HIGH);//backstop
>check(255, LOW, LOW, 127, HIGH, LOW);//backfwdhalf
>check(255, LOW, LOW, 255, HIGH, LOW);//backfwd
>check(127, LOW, LOW, 255, HIGH, LOW);//backhalffwd
>check(0, LOW, HIGH, 255, HIGH, LOW);//stopfwd
>check(127, HIGH, LOW, 255, HIGH, LOW);//fwdhalffwd }


Below is the code we used to achieve micro step operation. [Here](https://www.youtube.com/watch?v=G_4klbTj600 "Demonstration") is a link to the demonstration.
>//#define DEBUG
>// Motor winding pins
>#define A_DIR_PIN 12
>#define A_PWM_PIN 3
>#define B_DIR_PIN 13
>#define B_PWM_PIN 11
>#define FORWARD HIGH
>#define BACKWARD LOW
>#define TWO_PI 6.28318530718
>#define STEP_PERIOD 120
>#define MICROSTEP_DIVISIONS 8
>void setup() {
>// Configure pins for motor windings
>pinMode(A_DIR_PIN, OUTPUT);
>pinMode(A_PWM_PIN, OUTPUT);
>pinMode(B_DIR_PIN, OUTPUT);
>pinMode(B_PWM_PIN, OUTPUT);
>Serial.begin(9600);
>}
>void setPhase(double angle) {
>int aPWM = 255 * cos(angle); //winding A is the cosine wave
>int aDir = (aPWM > 0.0) ? FORWARD : BACKWARD;
>aPWM = abs(aPWM); //these two lines determine if it is negative, then make it not negative, as the motors do not accept negative numbers to make them run
>digitalWrite(A_DIR_PIN, aDir);
>analogWrite(A_PWM_PIN, aPWM);
>int bPWM = 255 * sin(angle); //winding B is the sine wave
>int bDir = (bPWM > 0.0) ? FORWARD : BACKWARD;
>bPWM = abs(bPWM);
>digitalWrite(B_DIR_PIN, bDir);
>analogWrite(B_PWM_PIN, bPWM);
>#ifdef DEBUG
>Serial.print("=================="); Serial.print(angle); Serial.println("==================");
>Serial.println();
>Serial.print("aPWM: "); Serial.println(aPWM);
>Serial.print("aDir: "); Serial.println(aDir);
>Serial.println();
>Serial.print("bPWM: "); Serial.println(bPWM);
>Serial.print("bDir: "); Serial.println(bDir);
>Serial.println();
>Serial.println("======================================================");
>#endif
>} //debugging just to see where it goes wrong
>void loop() {
>for (int microStep = 0; microStep < MICROSTEP_DIVISIONS; microStep++) {
>double phaseAngle = (double)microStep * TWO_PI / (double)MICROSTEP_DIVISIONS;
>setPhase(phaseAngle);
>delay(STEP_PERIOD / MICROSTEP_DIVISIONS);
>}
>}

<compare torque of different ops>

in conclusion, stepper motors are a versatile tool for mechanical movement via electromagnetism with their low cost and high 
Torque at low speeds and varying resoliton make them useful for a various number of fields, such as pumps and valves.

#**Lab 7: Arm mini-project**
===============

The last lab project we were given was to design, build and code a small robotic arm with 2 degrees of freedom.
It was to use servo motors in order to function and move.

Servo motors are small, energy-efficient motors. they are used in a variety of uses in many fields, with 
a large range of uses in remote control. Whilst they are energy efficicient, they output little torque.

They operate using gears to increase their oputput torque. At their core, servo motors house a small
DC motor. This motor spins very fast, but has little power. To counter this, the motor utilises gears
in an arrangement slows down the speed of the motor, but increases its torque. Using this method,
the work the motor outputs stays the same, as the formula for work is as follows:

> WORK = FORCE X DISTANCE

to keep the motor lightweight, the gears are normally manaufactured from plastic, however they can
be made from metal for motors that require increased strength.
Unlike a DC motor the roatational speed of the motor is slowed down by multiple gears. There is also 
a positional sensor is located on the final gear in order to constantly determine the servos rotation.
Using an on-board calculation, the servo can translate a numerical value to a desired position and can 
freely move to it.


Servo motors are controlled by PWM. The motor has a maximum and minimum pulse width, the width 
of the pulse determines how far the motor will move. 

An interesting feature of stepper motors is whilst power is supplied to them, they will resist
outside forces acting on the motor and will retain its current position. The maximum amount of force
they can resist is called their **Torque Rating**. However, the motor only knows what position to be in
due to the pulse it receives from the control line, thus removing power and therefore the pulses will
stop the motor resisting forces.


##Servo Control
------

The first part of building the arm was to first control the servo motors. Below is code for the servo motors
to sweep back and forth using Trigonometry waves. [Here](https://www.youtube.com/watch?v=T_9FXfGmq7Y) is a link
to the demonstration.


>#include <Servo.h>
>
>Servo servo1;
>
>Servo servo2;
>
>
>int count;
>
>int cosA;
>
>int sinB;
>
>void calc(){
>
>	cosA = cos(count * 0.01745329) * 180;
>
>	sinB = sin(count * 0.01745329) * 180;
>
>}
>
>void reset(){
>	cosA = 0;
>	sinB = 0;
>}
>
>void setup(){
>	servo1.attach(9);
>	servo2.attach(8);
>}
>
>void loop() {
>	for (count = 0; count <= 180; count += 1) {
>		calc();
>		writeto();
>	}
>	for (count = 180; count >= 0; count -= 1) {
>		calc();
>		writeto();
>	}
>}
>
>
>void writeto (){
>	servo1.write(cosA);
>	servo2.write(sinB);
>	delay(15);
>}
>

To build upon our servo motor control, we were given the task of controlling the servo motors
position using a potentiometer. Below is code to its operation and [here]() is a link to the demonstration.

>
>
>
>


##Designing the Arm
==============

Naturaly, the second part of the Arm process was to design the Arm. For this project, a few rough sketches gave us a few avenues to pursue
and we ended up deciding to build an arm with 2 revolute joints on the same plane for versatility and stability. The arm was intially designed in
Fusion 360, a CAD software program for students. It consisted of four pieces, the base, the first link, the second link and the end effector which was in 
this case a gripper.

From this point, the .Step files of the arm were individually exported to solidworks on the University computers. Next, using solidworks' Mate feature in its assembly 
category, joints were added to complete the arms assembly. this assembly file was then converted to an URDF and then a package was made for it to be inserted into a linux computer.

The URDF file recognized the joints implemented in Solidoworks but a few changes were made. For example, upper and lower limits were added to the joints as well as collision paramters,
this was in order to prevent the arm from assuming an incorrect position that would collide with itself. Below is the contents of Final.URDF, the final rendition of the urdf used to drive our arm:


```
<robot
  name="final">
<!--<link name="map">
	<visual>
	<origin xyz="0 0 0" rpy="0 0 0"/>
	<geometry>
		<box size="0.1 0.1 0.1"/>
	</geometry>
	</visual>
</link>  -->




<link
    name="baselink">
   <!-- <inertial>
      <origin
        xyz="0.0289573098297565 0.0395518547237048 0.0847632618649911"
        rpy="0 0 0" />
      <mass
        value="0.0597267797400795" />
      <inertia
        ixx="0"
        ixy="0"
        ixz="0"
        iyy="0"
        iyz="0"
        izz="0" />
    </inertial>-->
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://final/meshes/base_link.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.792156862745098 0.819607843137255 0.933333333333333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://final/meshes/base_link.STL" />
      </geometry>
    </collision>
  </link>
  <link
    name="link1">
    <inertial>
      <origin
        xyz="5.31220839886104E-11 0.0337219220577653 0.0267729199184993"
        rpy="0 0 1.57" />
      <mass
        value="0.03745547596626" />
      <inertia
        ixx="0"
        ixy="0"
        ixz="0"
        iyy="0"
        iyz="0"
        izz="0" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://final/meshes/link1.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.627450980392157 0.627450980392157 0.627450980392157 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://final/meshes/link1.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="joint1"
    type="revolute">
    <origin
      xyz="0.0018615 -0.062046 0.066282"
      rpy="1.57 0 0" />
    <parent
      link="baselink" />
    <child
      link="link1" />
    <axis
      xyz="0 0 1" />
    <limit
      effort="0"
      velocity="0" lower="-1.57" upper="1.57" />
  </joint>
  <link
    name="link2">
    <inertial>
      <origin
        xyz="-4.98101559998076E-12 0.0420081965687308 0.0275965656366171"
        rpy="0 0 0" />
      <mass
        value="0.045750001553657" />
      <inertia
        ixx="0"
        ixy="0"
        ixz="0"
        iyy="0"
        iyz="0"
        izz="0" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://final/meshes/link2.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.627450980392157 0.627450980392157 0.627450980392157 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://final/meshes/link2.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="joint2"
    type="revolute">
    <origin
      xyz="0 0.07 0.01"
      rpy="0 0 -1.2051" />
    <parent
      link="link1" />
    <child
      link="link2" />
    <axis
      xyz="0 0 1" />
    <limit
      effort="0"
      velocity="0" lower="-1.57" upper="1.57" />
  </joint>

  <link
    name="link3">
    <inertial>
      <origin
        xyz="0.0375 0.0100000000000002 0.0310919540229886"
        rpy="0 0 0" />
      <mass
        value="0.029" />
      <inertia
        ixx="0"
        ixy="0"
        ixz="0"
        iyy="0"
        iyz="0"
        izz="0" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://final/meshes/link3.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.627450980392157 0.627450980392157 0.627450980392157 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://final/meshes/link3.STL" />
      </geometry>
    </collision>
  </link>
  
<joint
    name="joint3"
    type="fixed">
    <origin
      xyz="-0.0375 0.145 0.0175"
      rpy="1.57 0 0" />
    <parent
      link="link2" />
    <child
      link="link3" />
    <axis
      xyz="0 0 0" />
  </joint>
<!--<joint name="mapjnt" type="fixed">
<origin xyz="0 0 0" rpy="0 0 0"/>
<parent link="map"/>
<child link="baselink"/>
</joint>
<joint name="jnt1" type="fixed">
<origin xyz="0 0 0" rpy="0 0 0"/>
<parent link="map"/>
<child link="link1"/>
</joint>
<joint name="jnt2" type="fixed">
<origin xyz="0 0 0" rpy="0 0 0"/>
<parent link="map"/>
<child link="link2"/>
</joint>-->
</robot>

````

As seen above, this URDF calls the .STL files of the arms design in itself, but still relates the models to one 
another via links and joints. During the process of implementing this URDF we encountered a few problems and
obstacles. One particular issue was relating the model to the world_frame,
the default frame RVIZ uses when it boots. There is no reference to the world or map frame in the URDF and as this piece of vestigial code
shows:

````
<!--<link name="map">
	<visual>
	<origin xyz="0 0 0" rpy="0 0 0"/>
	<geometry>
		<box size="0.1 0.1 0.1"/>
	</geometry>
	</visual>
</link>  -->

````
, we attempted to create a dummy link that would act as the base frame. Instead we found the best solution was to use the 
base link in the URDF as the fixed frame of the world, and this allowed RVIZ to visualize the arm correctly.

The final hurdle was when it came to use RVIZ to move the robot using the ROS package Joint_state_publisher and Robot_state_publisher,
RVIZ was unable to visualize the arm correctly and when Joint_state_publisher launched, it only featured an empty window.
One solution we tried is stated in this vestigial code shown:

````
<!--<joint name="mapjnt" type="fixed">
<origin xyz="0 0 0" rpy="0 0 0"/>
<parent link="map"/>
<child link="baselink"/>
</joint>
<joint name="jnt1" type="fixed">
<origin xyz="0 0 0" rpy="0 0 0"/>
<parent link="map"/>
<child link="link1"/>
</joint>
<joint name="jnt2" type="fixed">
<origin xyz="0 0 0" rpy="0 0 0"/>
<parent link="map"/>
<child link="link2"/>
</joint>-->
````
Above, we can see that we created a new set of joints that were parallel in parent and child links to the ones above,
but were all fixed. To our Surprise, this fixed the issue of the arm not visualizing, but did not fix the joint_state_publisher
launching with an empty window. But this did allow us to discover the problem lay in the code for the joints in the URDF file.
After consideration and Trial and Error fixes, we determined it was due to faulty limits stated in the URDF file, when this
was resolved the arm initialised and we were able to move the arm in RVIZ using sliders. 

##Controlling the Arm
======

to begin with, a few tutorials were presented to us to give us a better understanding of ROS's capability.
the first task was to run roscore in a terminal, below is the result:
[roscore](https://github.com/Jfishgoy/Journal.md/blob/master/roscoreterminal.png)

next was to create a rostopic then call it, the steps takne are documented below:
[first was to update ros with the arduino serial](https://github.com/Jfishgoy/Journal.md/blob/master/rosdepupdate.png)
[creating the topic](https://github.com/Jfishgoy/Journal.md/blob/master/stringpublish.png)
[showing the rostopic list](https://github.com/Jfishgoy/Journal.md/blob/master/stringlist.png)
[calling the topic](https://github.com/Jfishgoy/Journal.md/blob/master/rostopicecho.png)



 With the design of the arm in place, we set upon our next task of controlling the arm in linux's console terminal,
via arduino code.

sample code was provided to control the servo motor, it is shown below, ad [here]() is a video demonstrating
its functionality.

````
#include <Servo.h>
Servo myservo;
int pos = 0;

void setup() {
  myservo.attach(9); // attaches the servo on pin 9 to the servo object
}

void loop() {
  for (pos = 0; pos <= 180; pos += 1) {
    // in steps of 1 degree
    myservo.write(pos);
    delay(15);
  }
  for (pos = 180; pos >= 0; pos -= 1) {
    myservo.write(pos);
    delay(15);
  }
}
````

````

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <Servo.h>

using namespace ros;

NodeHandle nh;
Servo servo1;
Servo servo2;

void cb (const std_msgs::UInt16& msg){
  servo1.write(msg.data);
  servo2.write(msg.data);
}

void setup() {
  // put your setup code here, to run once:
  nh.initNode()
  nh.subscribe(sub);

  servo1.attach(9);
  servo2.attach(10);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(1);
}
````

In order to launch the file in RVIZ, a ".launch" file must be created. Using this launch file, several parameters and
nodes can be launched at once in order to refrain from several terminals. Below is the code for the launch file used.

````
<launch>
  <arg name="model" />
  <arg name="gui" default="False" />
  <param name="robot_description" textfile="$(find final)/urdf/final.urdf" />
  <param name="use_gui" value="$(arg gui)" />
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find final)/urdf.rviz" />
</launch>
````

rosrun can be run simply by using the code:

>$rosrun rviz rviz

below is the screenshot of launching RVIZ this way:
[launch](https://github.com/Jfishgoy/Journal.md/blob/master/rvizlaunch.png)

As we can see above, this code sets the robot_description to the URDF link. It also starts two nodes,
joint_state_publisher and robot_state_publisher to run alongside roscore and rviz.
below is a screenshot of the launch file starting:
[Final.launch]()

Below is a screenshot of the finished file in Rviz with joint state publisher 
[RVIZ visualised](https://github.com/Jfishgoy/Journal.md/blob/master/rvizjoints.png)






