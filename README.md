# IR+Ultrasonic+Laser Sensor DCC Emergency STOP

## Background
The [DGMF](https://danskgmodelforening.dk/) rail system has swing bridges for passenger passage, which can cause train systems to drive over the edge and fall to the floor. To counteract this type of potential accident, automatic micro switches have been installed to electrically disconnect the rails at a distance of between 125 – 185 cm.

However, the electrical disconnection is not sufficient on locomotives that have installed [PowerCap](/image/Massoth-PowerCap-micro.jpg), which is designed to power the decoder and motor even if the connection to the rails is disconnected. The same applies to tracks that end blindly or with end stops. The electrical disconnection also does not work sufficiently for larger trainsets that are reversing, or trainsets where the locomotive pushes cars forward at the rear.

There is a need for a new solution that works independently of the direction of travel, and thus where the locomotive is located. In the following, a digitally controlled solution is described, which is basically based on infrared passage measurement and [DCC E-stop](https://dccwiki.com/E-Stop) (emergency stop) via the [DiMAX Feed Back module](/image/Massoth-8170010-Dimax-FB-Feedback-module.jpg).

<br/>

![](/image/Swing%20bridge%20open%20and%20close.png)

## PowerCap
To ensure smooth, stable train operation and lights that do not flicker, it is recommended to install a so-called [PowerCap](/image/Massoth-PowerCap-micro.jpg), which consists of a little control electronics and some powerful electrolytic capacitors. It can be compared to a small accumulator. PowerCap is designed to take over when there is dirt on the rails and in case of a short-term lack of connection, for example, at switch tracks and more. When fully charged, PowerCap can keep a locomotive running for up to 30 seconds.

![](/image/Massoth-PowerCap-micro.jpg)

For analog operation, [Massoth](https://www.massoth.de/en/) recommends the largest possible buffer to ensure light & sound effects (Standing Noise) when the train is stationary.
It is possible to configure how many seconds  [PowerCap](/image/Massoth-PowerCap-micro.jpg) should take over in the event of an electrical interruption on the rails. CV 47 can be configured on the Massoth [eMOTION XLS](/image/massoth-emotion-xls-sound-decoder.jpg) between 1..240 in steps of 0.125 seconds per step. By default, CV 49 is set to 10 = 1.25 seconds. At first glance, one might think that it was so short that the train stops in time. But it only works at slow speeds, or where the interruption of the rails occurs several meters earlier.

Test experiments with CV 47 = 10 have shown that at medium speed (corresponding to 44 km/h in full scale) a small locomotive will travel a distance of about 130 cm. before it stops completely. At higher speeds (57 km/h) the locomotive will only stop after 165 cm. Longer locomotives with two sets of electrical pickup shoes will travel even further before coming to a complete stop. The same applies to heavy locomotives / train sets. Electrical disconnection of rails is therefore **not** sufficient when a decoder is supplemented with [PowerCap](/image/Massoth-PowerCap-micro.jpg).

## DCC Emergency Stop (E-stop)
The Massoth Central Station (DiMAX) and all Massoth DCC decoders support DCC Emergency Stop. The same applies to other types of decoders that support the [NMRA](https://dccwiki.com/NMRA) Digital Command Control Specifications.

E-STOP or Emergency Stop is a special [speed step](https://dccwiki.com/Speed_Steps) defined in the [NMRA](https://dccwiki.com/NMRA) Digital Command Control Specifications. Inside the [DCC Packet](https://dccwiki.com/Digital_Packet) is a set of bits reserved for speedstep information. If all of the bits are set to a value of zero (0000 for 28 and 14 speed steps) the addressed decoder moves to the stop position.
If the Least Significant Bit (S0) is set to one (0001), the value is the E-Stop command. E-Stop is a Broadcast command, meaning that all decoders will immediately change to STOP and all moving trains halt.

> [!NOTE]
Unlike some other suppliers, Massoth Central Station disconnects the power to the rails during emergency stops.

By pressing the Emergency Stop button on your [throttle](https://dccwiki.com/Throttle) the command station generates a broadcast packet with the E-Stop command and that is immediately transmitted to the [booster](https://dccwiki.com/Booster) and onto the rails. Everything in motion will immediately stop. The Standard states that the decoder shall immediately cease applying power to the motor.

## Reflection and Refraction concept
![](/image/Infrared%20break%20beam%20solution.png)

### Advantages 
* Cheap and effective for all types of locomotives and wagons.
* Works in both forward and reverse driving.
* Works immediately, unlike electrical disconnection.
* Works with both analog and digital control. 

### Disadvantages
* All locomotives stop immediately.
* Requires special design of control electronics.
* A lot of extra manual work.


## Functinal concept

![](/image/Functional%20concept.png)

## Chronological process overview
When a swing bridge is opened, the circuit is activated via a micro switch, and the green status LED is switched on and lights up constantly. The connected sensors measure whether there are train cars or locomotives running on the rails, and thus whether they are about to fall over the edge.

When the sensor beam is reflected (method 1) or broken (method 2), the following happens:
1. A [DCC](https://en.wikipedia.org/wiki/Digital_Command_Control) Emergency stop signal is sent, all trains stop and the connection to the rails is interrupted.
2. The status LED changes color from green to red and flashes.
3. The buzzer starts and switches between two frequencies (like the sound from an old fire engine).

This alarm state is “locked” until the green button = Reset is pressed. This is regardless of whether trains are removed or not [(bistable / flip-flop function)](https://en.wikipedia.org/wiki/Flip-flop_(electronics)). While waiting for the green button = reset to be pressed, the buzzer sounds the alarm for a short period. The red LED then continues to flash. After a while, the Buzzer alarm is repeated for a short period. This cycle continues until the green button is pressed = Reset. The status LED changes color to green = OK. If the green button is pressed while the train is still stopped, a frequency-escalating alarm sounds and the press is ignored. When the swing bridge closes, a micro switch is deactivated, which sends a signal to the circuit. The connection to the rails is re-established, a DCC cancellation of E-stop (reset) signal is sent, the monitoring is deactivated and the green status LED goes out.

![](/image/Chronological%20Process%20Overview.png)

<br/>

- [X] Arduino code ready to use with english inline program documentation :tada:
- [ ] Convert rest of 40 pages of documentation from Danish to English. 
