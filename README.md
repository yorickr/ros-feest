# ros-feest
Projectleden:

- Guus van Dongen
- Gilian Joossen
- Yorick Rommers
- Jeroen Berk

Voor het project gaan wij een laserpointer volgen met een camera gemonteerd op de pionier robot. Ook gaan we een gezicht volgen wanneer dit gedetecteerd wordt door de webcam van de laptop op de robot. Dit zal gebeuren middels de laptop die bovenop de laptop staat. Deze zal via een usb naar serial adapter aangesloten zijn met de pionier robot. Ook zal deze laptop met de wifi verbonden zijn en een server draaien vanuit ROS. Dit zal nodig zijn omdat er dan een cliënt met ROS kan verbinden met de master ROS-server. Op deze manier kunnen we gegevens uitwisselen tussen de robot en de cliënt die zich ergens anders bevindt. Daarnaast gaan we middels rosaria (wat zorgt voor de vertaling naar de aria chip in de pionier robot) de robot besturen middels besturing vanuit de camera maar ook vanaf een joystick. Qua hardware zijn er uiteindelijk dus twee laptops nodig, een usb naar seriële adapter, camera, lasterpointer, en een controller(joystick).

**OpenCV**

De camera zit op de robot bevestigd en kan een laserpointer volgen.

De webcam van de laptop op de robot kan een gezicht detecteren en volgen.

**Taken:**

OpenCV lasterpointer filteren **– Yorick**

OpenCV coordinaten vertalen naar besturing **– Jeroen**

Netwerkcommunicatie en logging **- Guus**

Joystick werking **– Gilian**

OpenCV face tracking **-**
