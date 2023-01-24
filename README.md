# Touch/MQTT controlled lights

Lights controlled by MQTT and a TP223 capacitive touch sensor by ESP01. Alternatively these lights can be controlled by capacative touch sensor only (No MQTT) using an ATTINY85 (easiest to use a Digispark Pro / Digistump). Obviously the capacative touch sensor can be replaced by any other button. Currently it's programmed for a falling edge transition.

The included website can be hosted locally and connects to the lights using websockets and mqtt. This requires a local MQTT broker to be all set up and running locally.

Since in my situation I have a completely shut firewall, not too much emphasis has been on security.

# Circuit board

The board is originally designed to accomodate the Digistump, but it will also connect to any ESP8266 (ESP01 is enough).

# Ghost touches

There's an intermittant issue with ghost touches when using the TP223 capacative touch sensors. Not sure if it is because of the steel boat the lights are installed in but in very rare circumstances the touch sensors "go rogue". This means they will output sustained touch events and this will keep the lights in the brightest setting. So to combat this there's a ghost touch detection routine that will temporarily disable all touch inputs until the situation has rectified itself. 

You can also solder a 47pF capacitor to the empty pads on the TP223 sensor module to make it less sensitive.

Some possible causes of this behaviour include:
- Insects near/on the sensor
- Certain atmospheric conditions
- Water or moisture
- Aliens, etc., etc.

# License

Copyright 2023 WJJPeterse

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.