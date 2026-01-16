# Antenne-Tracker
Ceci est le GitHub du Projet Antenne tracker . 



![IMG_4925](https://github.com/user-attachments/assets/7d43086a-3e6b-4c9a-a8f6-ec3d616ec3ed)



Matériel :

-Raspberry pi Zero 2W
-Elegoo power MB V2
-ULN2003APGAPG : Driver moteur pas à pas
-STEP-MOTOR : 2x 28BYJ-48
-MPU-6500 a changer vers Module 9 DoF BNO055 ADA4646 pour avoir un MAG 
-M10Q-5883 mateksys
-Holybro 433MHZ



Stucture :

-Main.py : point d’entrée. Configure les logs, instancie le Manager et exécute la boucle principale.

-States.py : machine à états (INIT / TRACKING / ERROR / SHUTDOWN).

-Sensor_Manager.py : orchestration capteurs (init, lecture, gestion d’erreurs, fermeture).

-DataClass.py : structures de données partagées (référence GPS, orientation tracker, solution de pointage).

-Tracker.py : calculs géodésiques (ECEF/ENU) et calcul azimut/élévation + extraction position cible depuis la télémétrie.

-Capteurs dans Sensor :

-GPS.py : lecture NMEA (GGA) via port série.

-MPU.py : lecture I2C + estimation roll/pitch/yaw (filtre complémentaire).

-Telemetry.py : connexion MAVLink (pymavlink), agrégation en “snapshot” des derniers messages reçus.

-Servo.py : (prévu) pilotage servo / moteur via GPIO (Raspberry Pi).

-Machine à états (logique de run)



Fonctionnement : 

La boucle de Main.py appelle un état à chaque itération :

-INIT (State_Init) :
Initialise les capteurs via manager.sensor_manager.init().

Si tout est OK → passe en TRACKING
Sinon → passe en ERROR

-TRACKING (State_Tracker) :

Lit les capteurs via manager.sensor_manager.read()
Récupère :
la position du tracker (GPS)
la position/attitude de la cible (snapshot télémétrie MAVLink)
Calcule le pointage :
extraction de la cible via telemetry_extract_target_lla() dans Tracker.py
calcul az/el via compute_az_el_from_positions() dans Tracker.py
Met à jour manager.pointing (azimuth/élévation/distances) pour un éventuel module de pilotage moteurs.

-ERROR (State_Error) :

Tente une récupération des capteurs via sensor_manager.error()
Cas particulier : si le GPS n’a pas de fix, le programme peut demander une référence manuelle (latitude/longitude/altitude) pour continuer le tracking depuis une position fixe.

-SHUTDOWN (State_Shutdown) :
Ferme proprement les capteurs (close()), puis quitte.



Données manipulées :

Les infos partagées sont stockées dans des dataclasses de DataClass.py :

-TrackerReference : référence GPS (automatique ou saisie manuelle).

-TrackerOrientation : orientation estimée (roll/pitch/yaw) depuis le MPU.

-PointingState : résultat du calcul (az_deg, el_deg, distance, source).

-Calcul du pointage (azimut/élévation)



Dans Tracker.py :

Conversion LLA → ECEF, puis ECEF → ENU (repère local du tracker)
Calcul :
az = atan2 (E,N) (ramené dans [0,360])
el = atan2(U,√E²+ N²)

Résultats : az_deg, el_deg, slant_range_m, ground_range_m



Exécution :

Connexion au Pi :

- ssh ESNA@2a05:6e02:1132:2310:3633:ef2d:1930:21b2

mp : 1111



Lancement du code : 

- cd Documents/Antenne_Tracker/

- python Main.py

- source environement-env/bin/activate



Config ports :

GPS : réglable via GPS.init(port=..., baud=...) dans GPS.py
Télémétrie : réglable via Telemetry.init(port=..., baud=...) dans Sensor_Manager.py



Dépendances (Python) :

pymavlink (MAVLink)
pyserial + pynmea2 (GPS NMEA)
smbus2 (I2C MPU, Raspberry Pi)
RPi.GPIO (servos/moteurs, Raspberry Pi)

Le MP est : 1111
ET l'utilisateur est : ESNA
Nom d'hôte : ESNA-Raspberry
l'adresse ipv6 est : 2a05:6e02:1132:2310:3633:ef2d:1930:21b2
