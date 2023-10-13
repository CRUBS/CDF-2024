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

Pour cela, nous allons utiliser l'IDE MPLAB X, qui est spécialement développé pour la programmation des PIC. 

Nous allons voir comment ont été programmés les différents éléments afin d'asservir le moteur ainsi que de communiquer avec d'autres contrôleurs  via bus CAN. Pour toute cette partie, il sera nécessaire de se référer à la [documentation du PIC](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU16/ProductDocuments/DataSheets/dsPIC33EPXXXGP50X-dsPIC33EPXXXMC20X-50X-and-PIC24EPXXXGP-MC20X-Family-Data-Sheet-DS70000657J.pdf) ainsi qu'aux documentations des différents modules.

<span id="PWM"><span>
### Génération d'une PWM

TODO

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

