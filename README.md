# Train_Emergency_Stop
IR+Ultrasonic+Laser Sensor DCC Emergency STOP

## Background
The [DGMF](https://danskgmodelforening.dk/) rail system has swing bridges for passenger passage, which can cause train systems to drive over the edge and fall to the floor. To counteract this type of potential accident, automatic micro switches have been installed to electrically disconnect the rails at a distance of between 125 – 185 cm.

However, the electrical disconnection is not sufficient on locomotives that have installed PowerCap, which is designed to power the decoder and motor even if the connection to the rails is disconnected. The same applies to tracks that end blindly or with end stops. The electrical disconnection also does not work sufficiently for larger trainsets that are reversing, or trainsets where the locomotive pushes cars forward at the rear.

There is a need for a new solution that works independently of the direction of travel, and thus where the locomotive is located. In the following, a digitally controlled solution is described, which is basically based on infrared passage measurement and DCC E-stop (emergency stop) via the DiMAX Feed Back module.
