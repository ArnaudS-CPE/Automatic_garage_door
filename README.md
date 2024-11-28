# Porte de garage automatique

## Module : Capteurs/actionneurs et prototypes

Ce projet implémente un système de gestion automatique de porte de garage basé sur ROS2 et une connection Bluetooth. Il permet de détecter un véhicule, lire sa plaque d'immatriculation, et contrôler l'ouverture/fermeture de la porte de manière automatisée. Le système utilise plusieurs nœuds ROS2 pour gérer la détection de voiture, la lecture de plaque, et la commande du servomoteur pour l’ouverture de la porte. 

Présentation PowerPoint disponible [ici](img/Presentation.pptx) 

## Table des Matières

- [Porte de garage automatique](#porte-de-garage-automatique)
  - [Module : Capteurs/actionneurs et prototypes](#module--capteursactionneurs-et-prototypes)
  - [Table des Matières](#table-des-matières)
  - [Auteurs](#auteurs)
  - [Application Android](#application-android)
  - [Structure des Nœuds](#structure-des-nœuds)
    - [1. BluetoothManager](#1-bluetoothmanager)
    - [2. car\_detection](#2-car_detection)
    - [3. ServoNode](#3-servonode)
    - [4. ToFNode](#4-tofnode)
    - [5. plate\_reader](#5-plate_reader)
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

![img](/img/schema.png)

Un télémètre infrarouge Sharp détecte la voiture lorsqu'elle s'approche de la porte. Quand la voiture est détectée, des leds s'allument pour éclairer la plaque, et l'Arduino envoir un message à la raspberry pi.



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

### 1. BluetoothManager

Ce nœud gère la connexion Bluetooth avec un appareil externe (ex: un téléphone) pour recevoir des commandes. Il peut ajouter ou supprimer une plaque d'immatriculation de la liste autorisée via un fichier JSON, selon les instructions reçues :

- `w:<valeur>` : écrit la plaque si elle n'est pas déjà présente dans le fichier JSON.
- `d:<valeur>` : supprime la plaque si elle est présente dans le fichier JSON.
- Si le message reçu est `"open"`, le nœud publie une commande sur le topic `door_control_command` pour ouvrir la porte.

### 2. car_detection

Ce nœud utilise une caméra pour détecter les voitures en capturant des images lorsque le signal est reçu via un capteur connecté via un port série. Si une voiture est détectée (signal `'a'` reçu), une image est capturée et publiée sur le topic `car_detected`.

### 3. ServoNode

Ce nœud contrôle les servos pour ouvrir et fermer la porte. Il souscrit au topic `door_control_command` et effectue l’action suivante :

- `open` : ouvre la porte en ajustant les angles des servos.
- `close` : ferme la porte.
Ce nœud publie également un message sur `car_coming_in` lorsque la porte est ouverte, indiquant l'arrivée d'un véhicule.

### 4. ToFNode

Ce nœud utilise un capteur ToF pour détecter si une voiture est complètement stationnée dans le garage. Lorsqu'un véhicule entre (`car_coming_in`), il vérifie la position du véhicule en lisant les données série du capteur ToF jusqu'à ce que le véhicule soit stationné.

### 5. plate_reader

Ce nœud utilise l'OCR pour lire la plaque d'immatriculation d'une voiture détectée. Il reçoit les images du topic `car_detected`, extrait le texte et publie la plaque sur le topic `plate` si une plaque valide est détectée.

### Graph

![img](/img/graph.png)

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

**Lancer launch file ROS2** :
   Lancez le `launch file` :

   ```bash
   sudo sdptool add --channel=22 SP
   ros2 launch proto_garage launch.py
   ```

## Fichiers Principaux

- **`bluetooth_connection.py`** : Gère les commandes Bluetooth et la gestion de la liste des plaques d'immatriculation.
- **`car_detection.py`** : Gère la détection de véhicule via une caméra.
- **`servo_node.py`** : Contrôle l'ouverture et la fermeture de la porte du garage.
- **`tof_node.py`** : Vérifie la présence du véhicule dans le garage.
- **`plate_reader.py`** : Lit et extrait la plaque d'immatriculation d'une image via OCR.

## Utilisation

Une fois tous les nœuds en cours d'exécution, le système est prêt à gérer automatiquement l'ouverture de la porte de garage. Lorsqu'une voiture est détectée :

1. La plaque est lue et vérifiée dans le fichier JSON pour vérifier l'accès.
2. Si l'accès est autorisé, la porte est ouverte.
3. Le capteur ToF vérifie si la voiture est bien garée et ferme la porte après un délai.

## Vidéo

![link]()

## Liens utiles

<https://community.appinventor.mit.edu/t/raspberry-pi-bluetooth-send-receive/59846/4>

<https://stackoverflow.com/questions/71341540/how-to-fix-installation-error-by-pybluez-error-on-subprocess>
