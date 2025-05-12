<h1>Hexapod</h1>

Ce projet présente un robot hexapode simulé, développé en Python. Il intègre des modules de cinématique, des constantes de configuration, et une interface de simulation.
<h2>📁 Structure du projet</h2>

    main_sim.py : 
    Script principal pour lancer la simulation.

    kinematics_empty.py : 
    Module de cinématique (à compléter).

    constants.py : 
    Fichier de configuration des constantes du robot.

    utils.py : 
    Fonctions utilitaires pour la simulation.

    phantomx_description/ : 
    Contient les fichiers de description du robot.

<h2>⚙️ Installation</h2>

Clone le dépôt :

    git clone https://github.com/Ahliko/Hexapod.git
    cd Hexapod

Installe les dépendances requises :

    pip install numpy pygame pybullet onshape-to-robot transforms3d scipy pipot

<h2>🚀 Utilisation</h2>

Pour lancer la simulation :

    python main_sim.py -m <simulation/robot> -p <port_série_du_robot>


Assure-toi que tous les modules nécessaires sont correctement installés et configurés.

<h2>🧠 Fonctionnalités</h2>

    Simulation d'un robot hexapode avec six pattes.

    Modules de cinématique pour le contrôle des mouvements.

    Configuration flexible via le fichier constants.py.