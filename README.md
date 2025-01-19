# Train_Emergency_Stop
IR+Ultrasonic+Laser Sensor DCC Emergency STOP

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

## Functinal concept

![](/image/Functional%20concept.png)

## Reflection and Refraction concept
![](/image/Infrared%20break%20beam%20solution.png)
