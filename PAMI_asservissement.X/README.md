# Asservissement PAMI

Ici se trouve le code pour asservir les moteurs des PAMIs.
L'asservissement se fait à l'aide d'un dsPIC33CK256MP502. Le pic reçoit les commandes moteur par I2C avec une adresse prévue par moteur. 
Le control du moteur se fait uniquement à l'aide des pins de choix de sens du pont en H, c'est à dire que la PWM est branchée sur l'un des deux pins  de choix de sens.
Pour plus de détails concernant la configuration, se référer au README.md de l'asservissement du moteur, la logique est la même.

L'asservissement des PAMIs est réalisé à l'aide d'un correcteur PI et non PID comme pour le robot principal car les moteurs des PAMIs ne permettent qu'une faible variation de vitesse et l'asservissement est surtout porté sur la qualité du suivi de la commande. Le coefficient dérivateur augmente l'erreur statique.