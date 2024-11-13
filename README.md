# Porte de garage automatique

## Module : Capteurs/actionneurs et prototypes

Ce projet implémente un système de gestion automatique de porte de garage basé sur ROS2 et Bluetooth. Il permet de détecter un véhicule, lire sa plaque d'immatriculation, et contrôler l'ouverture/fermeture de la porte de manière automatisée. Le système utilise plusieurs nœuds ROS2 pour gérer la détection de voiture, la lecture de plaque, et la commande des servos pour l’ouverture de la porte.

## Auteurs

- Théotime PERRICHET
- Tom RECHE
- Arnaud SIBENALER

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

## Installation

### Prérequis

1. **ROS2** - Installez ROS2 (Humble ou une autre distribution compatible).
2. **Python 3.x** - Assurez-vous d'avoir Python 3.x installé.

### Dépendances

Installez les dépendances Python répertoriées dans `requirements.txt` :

```bash
pip install -r requirements.txt
```

**Note :** `cv_bridge` nécessite une installation ROS2 correcte et peut nécessiter des étapes supplémentaires. Consultez [la documentation de cv_bridge](http://wiki.ros.org/cv_bridge) en cas de problèmes d'installation.

### Exécution

1. **Lancer ROS2** :
   Lancez le `ros2 daemon` :

   ```bash
   ros2 daemon start
   ```

2. **Lancer les nœuds** :
   - Vous pouvez démarrer les nœuds individuellement ou utiliser un fichier de lancement ROS2 pour démarrer plusieurs nœuds en même temps.

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


## Liens utiles

https://community.appinventor.mit.edu/t/raspberry-pi-bluetooth-send-receive/59846/4

https://stackoverflow.com/questions/71341540/how-to-fix-installation-error-by-pybluez-error-on-subprocess
