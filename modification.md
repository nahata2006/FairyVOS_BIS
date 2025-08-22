# FairyVOS - Modifications prévues sur ORB-SLAM3

## Objectif général
Adapter ORB-SLAM3 pour un fonctionnement optimisé en environnements à gravité variable (microgravité, vols paraboliques, etc.) dans le cadre du projet **FairyVOS**.

---

## Modifications prévues

### 1. Fusion de capteurs inertiels
- Ajouter la possibilité d’utiliser un **accéléromètre secondaire** (externe au système embarqué principal).  
- Méthode : introduire une étape de **fusion des données** entre l’IMU intégrée et l’accéléromètre secondaire.
- Hypothèse de travail :  
  \[
  a_{fusion} = a_{IMU} - a_{secondaire}
  \]
  afin de compenser les biais et perturbations liés à l’environnement de vol.

---

### 2. Communication capteurs
- Étudier un protocole **filaire ou faible interférence électromagnétique** entre l’ordinateur embarqué et le capteur secondaire.  
- Objectif : éviter toute perturbation du drone ou de l’avion porteur.  

---

### 3. Calibration
- Définir une procédure de **calibrage des accéléromètres** avant chaque vol.  
- Stocker les paramètres de calibration dans un fichier dédié (`calibration.yaml`).  

---

### 4. Intégration logicielle
- Modification des modules ORB-SLAM3 suivants :
  - `ORB_SLAM3/imu/` : adaptation des modèles de bruit et d’intégration des mesures.  
  - `ORB_SLAM3/src/System.cc` : ajout d’un mode FairyVOS pour activer la fusion de capteurs.  

---

## Étapes suivantes
- Écrire un module prototype `fairyvos_sensor_fusion.cpp`.  
- Tester la robustesse sur données simulées (gazebo / rosbag).  
- Documenter les résultats dans la section *Informatique* de l’encyclopédie.
