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

L'objectif de l'asservissement est de s'assurer que la vitesse de rotation du moteur sera bien celle qui est attendu. Pour cela, la présence d'un encodeur  est nécessaire afin d'avoir un retour quant à la vitesse de rotation du moteur puis adapter la consigne de tension pour atteindre la vitesse de rotation attendue. Le moteur sera contrôlé par un [pont en H L298N](https://www.sparkfun.com/datasheets/Robotics/L298_H_Bridge.pdf).


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

La valeur à mettre dans le registre `PTPER` pour choisir la période de la PWM se fait avec la formule suivante : 

<p align="center">
<img width="673" alt="image" src="https://github.com/CRUBS/CDF-2024/assets/77966063/3e0538dc-9c32-4963-b192-88dd0d6ac388">
</p>

Nous allons aussi définir une fonction *set_duty_cycle* pour modifier le rapport cyclique de la PWM.

```c
/*
 * Set the duty cycle of the PWM
 * @param duty: wanted duty cycle of the pwm
 * 0 <= duty <=1
 */
void set_duty_cycle(float duty)
{
    // duty must be between 0 and 1
    if(duty < 0) duty = 0.0;
    else if(duty > 1) duty = 1.0;
    
    // MDC = Duty cycle register
    // PTPER = Period register
    MDC = PTPER * duty;
}
```

Pour définir le sens de rotation du moteur, nous allons définir deux pins en tant que sortie dans le *Pin Manager*. Nous choisissons les pins RB12 et RB13. Dans le *Pin Module*, nous allons mettre l'un des deux pin en valeur haute au démarrage en cochant la case. Une fois le code généré, nous nous rendons dans le fichier main et créons une fonction pour définir le sens de rotation du moteur.

```c
/*
 * Set the rotating direction of the motor
 * @param clockwise: set the rotating direction of the motor clockwise
 * The rotating direction set here may not be really it depending how the motor is cabled
 */
void set_rotation_clockwise(bool clockwise)
{
    // Set or reset RB12 and RB13 
    LATBbits.LATB12 = clockwise;
    LATBbits.LATB13 = !clockwise;
}
```

<span id="Encodeur"><span>
### Lecture de l'encodeur

La PWM est maintenant générée et nous pouvons donc contrôler le moteur à différentes vitesses. Nous allons voir comment lire la vitesse de rotation du moteur à partir de l'encodeur. Pour cela, un [module QEI](https://ww1.microchip.com/downloads/en/DeviceDoc/70000601c.pdf) (Quadrature Encoder Interface) est disponible sur les dsPIC, mais ne peut pas être défini depuis MCC.

Nous commençons par définir, dans le *Pin Manager*, les pins QEA et QEB sur les pins RB10 et RB11 en tant qu'entrées. Ces pins ont été choisis car ils sont tolérant jusqu'à 5 V et notre encodeur renvoie une tension de 5 V. Une fois le code généré, nous nous rendons dans le fichier main et créons une fonction pour initialiser le module QEI.

```c
/*
 * Initilisation of the QEI
 */
void init_QEI(void)
{
    RPINR14 = 0x2a2b; // Set QEI on RB10 and RB11 (pins 21 and 22)
    
    // Set parameters
    QEI1CONbits.CCM    = 0; // Counter Control Mode Selection bits set as x4 mode
    QEI1CONbits.INTDIV = 7; // Timer clock prescaler set as 1:128
    QEI1CONbits.IMV    = 0; // Index match value
    QEI1IOCbits.FLTREN = 0; // Deactivate filter
    QEI1CONbits.PIMOD  = 0; // Position counter is unaffected by the Index input
    QEI1IOCbits.SWPAB  = 0; // Don't swap QEA and QEB
    QEI1CONbits.QEIEN  = 1; // Enable QEI module
}
```

Une fois la fonction exécutée, la lecture de la vitesse de rotation du moteur sera réalisée. La variable `POS1CNTL` sera incrémentée à chaque impulsion de l'encodeur. Le traitement sera vu dans la partie [Utilisation des timers](#Timers).

<span id="Timers"><span>
### Utilisation des timers

Nous allons paramétrer un premier timer qui lèvera une interruption toutes les 12 ms afin de procéder à l'asservissement en fonction de la vitesse de rotation du moteur durant les 12 dernières millisecondes. Pour configurer le timer, nous allons utiliser MCC où depuis la fenêtre *Device Resources*, nous ajoutons le timer1. Le paramétrage est montré dans l'image suivante : 

<p align="center">
<img width="600" alt="image" src="https://github.com/CRUBS/CDF-2024/assets/77966063/b79855ab-705a-4ce3-ab93-245d6f091c04">
</p>

Une fois le code générer, nous modifions le fichier main. Voici le code à ajouter :

```c
#include "mcc_generated_files/tmr1.h"

// 12 = nb points coder ; 4 because the QEI mode is x4 ; angle in radians = 0.1309
#define ANGLE_CODER 360.0 / 12.0 / 4.0 * 3.1415926535897932384626433 / 180 
#define TIME_INTERVAL 0.01 // s

// 0.01 == time between 2 calls of the timer interrupt
const float rotating_speed_coef = ANGLE_CODER / TIME_INTERVAL;

int old_position = 0; // Previous position of the encoder

// PID variables
const int kp = 10, ki = 20; const float kd = 0.3; // Coef PID
volatile int previous_error = 0.0, integral = 0.0;
volatile int rotating_speed_target = 0; // rad/s

/*
 * Callback function called by the timer1 interrupts each 10 ms 
 * for calculating the rotating speed of the motor
 */
void speed_rotation_measure()
{
    IFS0bits.T1IF = 0;   // Clear timer 1 interrupt flag
    
    int current_position = (int) POS1CNTL; // Get the pulse count
    
    // Calculate the rotating speed in rad/s ; 
    // 0.01 being the time between to call of the function in s
    // Around 670 rad/s at max speed
    int rotating_speed = (current_position - old_position) * rotating_speed_coef;
    
    old_position = current_position;
    
    speed_count += rotating_speed;
    speed_measure_count ++;
    
    control_motor_speed(rotating_speed, TIME_INTERVAL); // Enslave
}
```

En définissant aussi les fonctions *set_rotating_speed_target* et *control_motor_speed* :

```c
/*
 * Enslave the motor to rotate at the speed defined by rotating_speed_target
 * depending on the current speed 
 * @param speed: current rotating speed of the motor in rad/s
 * @param time_interval: time between two controls
 */
void control_motor_speed(int speed, float time_interval)
{
    // Calculate the error between the target speed and current speed
    int error = rotating_speed_target - speed;
    if(rotating_speed_target < 0) error = -error;
    
    // Calculate the proportional term
    int proportional = kp * error;
    
    // Calculate the integral term
    integral += ki * error * time_interval;
    
    // Calculate the derivative term
    float derivative = kd * (error - previous_error) / time_interval;
    
    // Change the rotating speed
    set_duty_cycle((float) (proportional + integral + derivative) / 670.0);
    
    previous_error = error; // Update the error
}

/*
 * Set the rotating speed target of the motor.
 * @param target: wanted rotating speed of the motor, 
 * if <0, the motor rotate in the other direction
 */
void set_rotating_speed_target(int target)
{
    rotating_speed_target = target;
    
    // Set rotating direction
    set_rotation_clockwise(target > 0);
    
    // Reset
    integral = 0;
    previous_error = 0;
}
```

Il n'y a plus qu'à ajouter les lignes suivantes dans le main afin de lancer l'asservissement du moteur à la vitesse demandée : 

```c
TMR1_SetInterruptHandler(&speed_rotation_measure); 
set_rotating_speed_target(600);
TMR1_Start();
```

En rentrant une valeur négative en paramètre de la fonction ```set_rotating_speed```, le moteur sera asservi pour tourner dans l'autre sens.

Nous allons configurer un second timer afin de faire un retour quant à la vitesse du moteur durant les 100 dernières millisecondes. La configuration est sensiblement la même que pour le timer1, mais il faut changer le *Prescaler* pour *1:8* afin de pouvoir configurer une période de 100 ms.

Nous nous rendons maintenant dans le *Interrupt Module* et nous définissons une priorité de 6 pour le timer1 et une priorité de 5 pour le timer2. Nous mettrons la priorité maximale pour la réception de message par bus CAN par la suite.

Une fois le code généré, nous ajoutons le code suivant au fichier main (ne pas oublier de configurer la fonction de callback et de démarrer le timer dans la fonction main) :

```c
int speed_count = 0; // Sum of the speed
uint8_t speed_measure_count = 0; // Number of times speed_rotation_measure is called

/*
 * Callback function called by the timer2 interrupts each 100 ms
 * for sending the average rotating speed of the motor
 * Toggle the state of the led on pin RB5 at each call
 */
void send_average_speed()
{
    
    //printf("%d\n", speed_count / speed_measure_count);
    
    CAN_MSG_OBJ msg;
    if(CAN_CONFIGURATION_MODE == CAN1_OperationModeGet())
    {
        if(CAN_OP_MODE_REQUEST_SUCCESS == CAN1_OperationModeSet(CAN_NORMAL_2_0_MODE))
        {
            msg.msgId = 0x1FFFF;
            msg.field.frameType = CAN_FRAME_DATA;
            msg.field.idType = CAN_FRAME_EXT;
            msg.field.dlc = CAN_DLC_8;
            msg.data = 1;

            CAN1_Transmit(CAN_PRIORITY_HIGH, &msg);
        }
    }
    
    toggle_led_state();
    
    speed_count = 0;
    speed_measure_count = 0;
}
``` 

<span id="CAN"><span>
### Utilisation du bus CAN

TODO

<span id="Branchements"><span>
## Branchements du PIC

La configuration minimale des branchements du PIC est montrée dans l'image ci-dessous avec les branchements avec le Pickit 3 à gauche. Ce dernier permet de programmer le PIC.

<p align="center">
<img src="https://github.com/CRUBS/CDF-2024/assets/77966063/04a4bd4b-d3e9-432b-a8ee-341795e5c27e">
</p>

TODO

