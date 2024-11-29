# Porte de garage automatique

## Module : Capteurs/actionneurs et prototypes

Ce projet implémente un système de gestion automatique de porte de garage basé sur ROS2 et une connection Bluetooth avec une application mobile. Il permet de détecter un véhicule, lire sa plaque d'immatriculation, et contrôler l'ouverture/fermeture de la porte de manière automatisée en fonction de la plaque lue. Une application doit également pouvoir contrôler manuellement la porte et ajouter/supprimer des plaques à autoriser.
Le système utilise plusieurs nœuds ROS2 pour gérer la détection de voiture, la lecture de plaque, et la commande du servomoteur pour l’ouverture de la porte. 

Présentation PowerPoint disponible [ici](img/Presentation.pptx).

## Table des Matières

- [Porte de garage automatique](#porte-de-garage-automatique)
  - [Module : Capteurs/actionneurs et prototypes](#module--capteursactionneurs-et-prototypes)
  - [Table des Matières](#table-des-matières)
  - [Auteurs](#auteurs)
  - [Maquette](#maquette)
  - [Application Android](#application-android)
  - [Structure des Nœuds](#structure-des-nœuds)
    - [1. BluetoothManager](#1-bluetoothmanager)
    - [2. car\_detection](#2-car_detection)
    - [3. ServoNode](#3-servonode)
    - [4. ToFNode](#4-tofnode)
    - [5. plate\_reader](#5-plate_reader)
    - [6. plate\_checker](#6-plate_checker)
    - [Graph](#graph)
  - [Installation](#installation)
    - [Prérequis](#prérequis)
    - [Dépendances](#dépendances)
    - [Exécution](#exécution)
  - [Fichiers Principaux](#fichiers-principaux)
  - [Utilisation](#utilisation)
  - [Vidéo](#vidéo)
  - [Liens utiles](#liens-utiles)

## Auteurs

- [Théotime PERRICHET](https://github.com/TheoTime01)
- [Tom RECHE](https://github.com/TomRecheEln)
- [Arnaud SIBENALER](https://github.com/ArnaudS-CPE)

## Maquette

Pour tester notre prototype, nous avons réalisé une maquette à échelle réduite. 

![img](/img/maquette.JPG)


La maquette est composée d'un boitier contenant un télémètre infrarouge Sharp et deux leds, connectés à une Arduino, permettant de détecter un véhicule, et d'éclairer sa plaque d'immatriculation.
Lorsque qu'un voiture est détectée, l'Arduino envoie un message vers une Raspberry Pi 4, qui déclenche la capture d'une image de la plaque avec une webcam. La Raspberru Pi traite l'image et ouvre ou non la porte. 
La porte s'ouvre avec un servomoteur directement connecté à la Raspberry Pi.
Un capteur temps de vol détecte la distance entre la voiture et le mur, et une ESP32 équipée d'un écran affiche cette distance et déclenche la fermeture de la porte lorsque la voiture est bien garée.
Une application sur une tablette Android connectée en Bluetooth à la Raspberry Pi permet de contrôler manuellement la porte et d'ajouter ou supprimer des plaques d'immatriculation de la base de données.

Chaque élément du système (à l'exception du servomoteur) est connecté à la Raspberry Pi en USB, pour avoir l'alimentation et le transfert de données dans un seul cable.

![img](/img/schema.png)


### Impressions 3D réalisées :
- Boitier en 2 pièces contenant le télémètre Sharp et les leds
![img](/img/boitier.JPG)

- Boitier de fixation du servomoteur
![img](/img/servo.JPG)

- Charnières de la porte (modèle trouvé sur Internet)
![img](/img/charniere.JPG)


## Application Android

![img](/img/telephone-app.png)

Cette [application](app/p9B3i_enviar_recibirRaspBerry3.aia) a été développée sur [App Inventor](https://appinventor.mit.edu/).

**Utilisation**

 - Se connecter en Bluetooth du téléphone/tablette à l'ordinateur.
 - Cliquer sur le boutton *Start Connect*
 - Cliquer sur *Open door* pour ouvrir la porte du garage et *Close door* pour la fermer
 - Il est possible de supprimer et de rajouter des plaques d'immatriculation a la base de données json
 - Cliquer sur *Stop Connect* pour déconnecter l'application

## Structure des Nœuds

### Graph

![img](/img/graph.png)

### 1. BluetoothManager

Ce nœud gère la connexion Bluetooth avec un appareil externe (ex: un téléphone) pour recevoir des commandes. Il peut ajouter ou supprimer une plaque d'immatriculation de la liste autorisée via un fichier JSON, selon les instructions reçues :

- `w:<valeur>` : écrit la plaque si elle n'est pas déjà présente dans le fichier JSON.
- `d:<valeur>` : supprime la plaque si elle est présente dans le fichier JSON.
- Si le message reçu est `"open"`, le nœud publie une commande sur le topic `door_control_command` pour ouvrir la porte.

### 2. car_detection

Lorsqu'un caractère est reçu sur un port série (i.e. lorsqu'une voiture est détectée devant la porte), le nœud capture une image de la plaque d'immatriculation avec une webcam, et publie l'image sur le topic `car_detected`.

### 3. ServoNode

Ce nœud contrôle le servomoteur pour ouvrir et fermer la porte. Il souscrit au topic `door_control_command` et effectue l’action suivante :
- `open` : ouvre la porte en ajustant les angles des servos.
- `close` : ferme la porte.

Ce nœud publie également un message sur `car_coming_in` lorsque la porte est ouverte, indiquant l'arrivée d'un véhicule.

### 4. ToFNode

Ce nœud communique via un port série avec l'ESP32 pour activer l'affichage de la distance du véhicule au mur. Lorsque le véhicule est complètement stationné, l'ESP32 renvoie un message pour indiquer au nœud de refermer la porte.

### 5. plate_reader

Ce nœud utilise l'OCR (avec la bibliothèque Python _EasyOCR_, et un modèle pré-entraîné fourni) pour lire la plaque d'immatriculation d'une voiture détectée. Il reçoit les images du topic `car_detected`, en extrait le texte, le traite pour s'assurer qu'il s'agit bien d'un plaque d'immatriculation, et publie la plaque sur le topic `plate` si une plaque valide est détectée.

### 6. plate_checker

Ce nœud reçoie les plaques d'immatriculation lues sur le topic `plate`, et vérifie si elle est présente dans un fichier JSON contenat les plaques autorisées à entrer. Si la plaque est autorisée, le nœud envoie un message pour ouvrir la porte sur le topic `door_control_command`.

## Installation

### Prérequis

1. **ROS2** - Installez ROS2 Humble.
2. **Python 3.x** - Assurez-vous d'avoir Python 3.x installé.

### Dépendances

**Dépendances Bluetooth**

```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install bluetooth
sudo apt-get install bluez
sudo apt-get install python-bluez
```

Installez les dépendances Python répertoriées dans `requirements.txt` :

```bash
pip install -r requirements.txt
```

**Note :** `cv_bridge` nécessite une installation ROS2 correcte et peut nécessiter des étapes supplémentaires. Consultez [la documentation de cv_bridge](http://wiki.ros.org/cv_bridge) en cas de problèmes d'installation.

### Exécution

**Téléverser les scripts Arduino** :
  On doit téléverser les scripts Arduino correspondants dans l'Arduino Uno et l'ESP32, via l'IDE Arduino. Une fois téléversés, il n'est plus nécessaire de téléverser les scripts.

**Lancer launch file ROS2** :
   Lancez manuellement le `launch file` :

   ```bash
   sudo sdptool add --channel=22 SP
   ros2 launch proto_garage launch.py
   ```

  On a configuré la Rapsberry Pi pour que le `launch file` et la commande de configuration du bluetooth se lancent automatiquement à son lancement.

## Fichiers Principaux

- **`bluetooth_connection.py`** : Gère les commandes Bluetooth et la gestion de la liste des plaques d'immatriculation.
- **`car_detection.py`** : Gère la détection de véhicule via une caméra.
- **`servo_node.py`** : Contrôle l'ouverture et la fermeture de la porte du garage.
- **`tof_node.py`** : Vérifie la présence du véhicule dans le garage.
- **`plate_reader.py`** : Lit et extrait la plaque d'immatriculation d'une image via OCR.
- **`plate_checker.py`** : Vérifie si les plaques lues sont autorisées ou non.

- **`car_detect.ino`** : Script Arduino pour détecter les véhicules, allumer les leds et evoyer un caractère à la Raspberry.
- **`[sketch_nov7a.ino]`** : Script Arduino pour l'ESP32 affichant la distance mesurée par un capteur TOF lorsque qu'un message est reçu, et revoie un messagelorsque la voiture est suffisement proche du mur.

## Utilisation

Une fois tous les nœuds en cours d'exécution, le système est prêt à gérer automatiquement l'ouverture de la porte de garage. 

1. Lorsqu'une voiture s'approche suffisement de la porte, elle est détectée par le télémètre infrarouge, les deux leds s'allument et une image de la voiture est capturée par la Raspberry Pi.
2. La plaque est lue, et si elle correspond à une plaque autorisée, la porte est ouverte.
3. Le capteur ToF permet d'afficher la distance de la voiture au mur, et de vérifier si elle est bien garée pour déclencher la fermeture de la porte après un délai.

## Vidéo

![link]()

## Liens utiles

<https://community.appinventor.mit.edu/t/raspberry-pi-bluetooth-send-receive/59846/4>

<https://stackoverflow.com/questions/71341540/how-to-fix-installation-error-by-pybluez-error-on-subprocess>
