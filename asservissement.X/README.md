[comment]: <> (Projet réalisé en octobre 2023)
[comment]: <> (Calcul du PID pour l'asservissement par Léon COLLÉN)
[comment]: <> (PCB réalisé par Tristan ROUZIC)
[comment]: <> (Programmation réalisée par Axel TRÉMAUDANT)
[comment]: <> (Avec l'aide de Philippe GICQUEL)

# Asservissement moteur DC

## Table des matières

* [Introduction](#Introduction)
* [Création du PID](#PID)
* [Programmation du PIC](#Programmation)
    * [Génération d'une PWM](#PWM) 
    * [Lecture de l'encodeur](#Encodeur)
    * [Utilisation des timers](#Timers)
    * [Utilisation du bus CAN](#CAN)
* [Branchements du PIC](#Branchements)

<span id="Introduction"><span>
## Introduction

Ici se trouve le code pour l'asservissement en vitesse des moteurs DC du robot. L'asservissement est géré par un [dsPIC33EP256MC502](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU16/ProductDocuments/DataSheets/dsPIC33EPXXXGP50X-dsPIC33EPXXXMC20X-50X-and-PIC24EPXXXGP-MC20X-Family-Data-Sheet-DS70000657J.pdf) et est programmé via l'IDE MPLAB X.

<p align="center">
<img src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcT5ZJrTMOmJCWcdM6B0RgZzJ6M4O6GNavg-bY3bbremY-9b8lxuQrbD-xsGVfcEp-8HYUI&usqp=CAU" alt="drawing" height="150">
<img src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcS5NV3uVCUAZrtTarMT_Dgnukuh14O-aW9hdIFImqMjc2ezAOAG9l8v8b2ALXUv77fiyNg&usqp=CAU" alt="drawing" height="150">
</p>

L'objectif de l'asservissement est de s'assurer que la vitesse de rotation du moteur sera bien celle qui est attendu. Pour cela, la présence d'un encodeur  est nécessaire afin d'avoir un retour quant à la vitesse de rotation du moteur puis adapter la consigne de tension pour atteindre la vitesse de rotation attendue.


L'asservissement est réalisé avec un moteur DC allant à 156 tours/min en sortie de réducteur et à 6450 tours/min avant réducteur. L'encodeur utilisé est un encodeur incrémental 12 points. Vous pouvez trouver à [ce lien](https://www.posital.com/fr/produits/interface-de-communication/incremental/incremental-encoder.php) une explication du fonctionnement d'un encodeur. Il est à noter que notre encodeur ne possède pas de signal zéro (ou home).

<span id="PID"><span>
## Création du PID

TODO

<span id="Programmation"><span>
## Programmation du PIC

Le PID pour l'asservissement du moteur a été réalisé théoriquement. Il faut désormais programmer le PIC.

Pour cela, nous allons utiliser l'IDE [MPLAB X](https://www.microchip.com/en-us/tools-resources/develop/mplab-x-ide), qui est spécialement développé pour la programmation des PICs. 

Nous allons voir comment ont été programmés les différents éléments afin d'asservir le moteur ainsi que de communiquer avec d'autres contrôleurs  via bus CAN. Pour toute cette partie, il sera nécessaire de se référer à la [documentation du PIC](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU16/ProductDocuments/DataSheets/dsPIC33EPXXXGP50X-dsPIC33EPXXXMC20X-50X-and-PIC24EPXXXGP-MC20X-Family-Data-Sheet-DS70000657J.pdf) ainsi qu'aux documentations des différents modules.

Pour programmer le PIC, il faut aussi installer le compilateur. Pour un dsPIC33, il nous faut le compilateur 16 bits [XC16](https://www.microchip.com/en-us/tools-resources/develop/mplab-xc-compilers).

<span id="PWM"><span>
### Génération d'une PWM

Nous allons commencer par voir comment a été paramétrée la PWM sur le PIC.

La première étape est de démarrer **MCC**, qui est une interface graphique pour aider à la configuration des registres.
Pour démarrer MCC, il suffit de cliquer sur *Tools* dans la barre des menus puis *Embedded* et enfin *MPLAB Code Configurator*. Lors du premier démarrage il vous sera demandé de choisir un type de contenu, choisissez *MCC classic*. Ensuite cliquez sur *Finish*. Le lancement de MCC prend ensuite un certain temps. 

Finalement, vous devriez avoir une interface ressemblant à celle ci-dessous. Avec la zone centrale où l'on va réaliser le paramétrage, le panneau à droite où le PIC est représenté avec ses différents périphériques. En bas se trouve la zone que nous utiliserons pour sélectionner les différents pins et à gauche, nous avons un panneau nous permettant de sélectionner différents modules.

<p align="center">
<img src="https://github.com/CRUBS/CDF-2024/assets/77966063/2b1bdcad-bb9e-4f56-a2aa-d18150acd616">
</p>

Nous allons maintenant configurer le système pour notre utilisation. Pour cela, nous allons nous rendre dans le *System Module* puis configurer comme montré sur l'image ci-dessous. Nous choisissons l'oscillateur interne au PIC et mettons le postscaler à 1:1.

<p align="center">
<img src="https://github.com/CRUBS/CDF-2024/assets/77966063/81d44ebf-736a-412c-9094-03da5f7360db" alt="drawing" width="400">
</p>

Nous passons maintenant dans l'onglet *Registers* du *System Module* afin de procéder à un paramétrage plus fin pour permettre le fonctionnement de la PWM et du module QEI. Il faut régler les registres définis ci-dessous sous peine de non fonctionnement de la PWM.

<p align="center">
<img src="https://github.com/CRUBS/CDF-2024/assets/77966063/435752a3-779a-4687-9746-b5badc8bf519" alt="drawing" width="400">
<img src="https://github.com/CRUBS/CDF-2024/assets/77966063/729b6c2c-08e0-40f3-9704-3bec82eda92c" alt="drawing" width="400">
</p>

Il serait possible théoriquement possible d'utiliser le module PWM de MCC, mais je ne suis jamais parvenu à faire fonctionner la PWM depuis le module. Nous allons donc configurer la PWM manuellement.

Tout d'abord il faut définir le pin de PWM. En se référent à la doc, les pins RB10 à RB15 peuvent être utilisés pour générer une PWM. Nous allons utiliser le pin RB15. Il faut définir le pin en tant que sortie. Pour cela, se rendre dans le *Pin Manager: Grid View* en bas de l'écran. Dans la section *Pin Module*, sélectionner le pin 15 du port B en tant que sortie, la case doit devenir verte. 

Dans la section *Clock*, il est possible de désactiver le pin CLKO car il ne va pas nous servir.

Les pins activés s'affichent aussi dans le *Pin Manager: Package View* à droite.

Cliquez sur le bouton *Generate* en haut à gauche pour générer le code comme configuré précédemment.

Un fichier main.c a été généré et nous allons le modifier pour faire fonctionner la PWM. Se référer à la [documentation PWM](https://ww1.microchip.com/downloads/en/DeviceDoc/dsPIC33-PIC24-FRM-High-Speed-PWM-DS70000645.pdf) pour plus de détails quant au paramétrage.

Nous allons créer une fonction *init_PWM* pour initialiser la PWM. Cette fonction sera à appeler dans le main et la PWM fonctionnera. Nous définissons une PWM à 2 kHz avec un rapport cyclique de 50 %. 

```c
/*
 * Initialisation of the PWM 
 */
void init_PWM()
{    
    /* Set PWM Period on Primary Time Base */
    PTPER = 3684; // 500 us

    /* Set Phase Shift */
    PHASE1 = 0;

    /* Set Duty Cycles */
    MDC = 1842; // 50 %

    /* Set Dead Time Values */
    DTR1 = 0;
    ALTDTR1 = 0;

    /* Set PWM Mode to Push-Pull, swap for having a 'high' duty cycle
     * and enable only pwm1 on pin RB15
     */
    IOCON1 = 0x4C02;

    /* Set PWM Mode to Independent */
    PWMCON1 =  0x0100;

    /* Configure Faults */
    FCLCON1 = 0x0003;

    /* 1:1 Prescaler */
    PTCON2 = 0x0000;
    
    // Enable only pwm on pin RB15
    IOCON2 = 0;
    IOCON3 = 0;

    /* Enable PWM Module */
    PTCON = 0x8000;
}
```

Nous allons aussi définir une fonction *set_duty_cycle* pour modifier le rapport cyclique de la PWM.

```c
/*
 * Set the duty cycle of the PWM
 * @param duty: wanted duty cycle of the pwm
 * 0 <= duty <=1
 */
void set_duty_cycle(double duty)
{
    // MDC = Duty cycle register
    // PTPER = Period register
    
    MDC = PTPER * duty;
}
```

<span id="Encodeur"><span>
### Lecture de l'encodeur

TODO

<span id="Timers"><span>
### Utilisation des timers

TODO

<span id="CAN"><span>
### Utilisation du bus CAN

TODO

<span id="Branchements"><span>
## Branchements du PIC

TODO

