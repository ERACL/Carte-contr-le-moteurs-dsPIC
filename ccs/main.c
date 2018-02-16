/* 
 * File:   main.c
 * Author: Alexandre MARSONE
 *
 * Created on 30 novembre 2016, 13:03
 */

////////////////////////////////////////////////////////////////////////////////
//INCLUDES
////////////////////////////////////////////////////////////////////////////////

#include <33FJ128MC802.h>//on importe le module spécifique à notre DSPic
#include <stdio.h>// on importe un truc
#include <stdlib.h>// on importe un truc
#include <math.h>// on importe les fonctions mathématiques

////////////////////////////////////////////////////////////////////////////////
//FUSES
////////////////////////////////////////////////////////////////////////////////

#fuses ICSP1      // ICD uses PGC1/PGD1 pins
#fuses NOJTAG     // JTAG disabled
#fuses DEBUG      // Debug mode for use with ICD
#fuses NOWDT      // Watchdog timer enabled/disabled by user software
#fuses HS,NOPROTECT
//#fuses NOALTI2C1  // I2C1 mapped to SDA1/SCL1 pins

////////////////////////////////////////////////////////////////////////////////
//USE
////////////////////////////////////////////////////////////////////////////////

#use delay(clock=10000000)//pour pouvoir manipuler le temps
#USE I2C(SLAVE, SCL=PIN_B8, SDA=PIN_B9,ADDRESS=0x21, FORCE_HW)

////////////////////////////////////////////////////////////////////////////////
//PINS
////////////////////////////////////////////////////////////////////////////////

//on sélectionne les pins des encodeurs
#pin_select QEA1=PIN_B10
#pin_select QEB1=PIN_B7
#pin_select INDX1=PIN_B11 
#pin_select QEA2=PIN_B5 
#pin_select QEB2=PIN_B4 
#pin_select INDX2=PIN_B6

////////////////////////////////////////////////////////////////////////////////
//DEFINE
////////////////////////////////////////////////////////////////////////////////

#define MAX(x,y)  ((x > y) ? x : y) //on définit les fonctions min et max
#define MIN(x,y)  ((x < y) ? x : y)

////////////////////////////////////////////////////////////////////////////////
//VARIABLES GLOBALES
////////////////////////////////////////////////////////////////////////////////

//Tout se fait actuellement en cm. Il faudra convertir en mm. BOF niveau unites...

// Distance et angle de consigne a parcourir.
int16 distancec=0;
int16 anglec=0;
float32 anglec_rad=0;

int8 etat_robot=0; // 0 : est en attente d'une nouvelle consigne. 1 : est en mouvement. 2 : est en pause.
int8 ret_etat_robot=100; // Retient l'etet du robot lors d'une pause. 
int1 set_donnees=0;

float32 x=0;// l'abcisse du robot 
float32 y=0;// l'ordonnée du robot
float32 teta=0;// l'angle du robot en radians, compte positivement vers la droite
int16 tetadeg=0; // teta en degrés (plus pratique à lire qu'un angle en rad). N'est utilise que pour la communication avec le PI.
int16 puissancec=0;

// Les coordonnes que la raspberry nous donne (avec tetadeg)
int16 xpi=0;// l'abcisse du robot 
int16 ypi=0;// l'ordonnée du robot

int16 nd=0;// le nombre de "tics" envoyés par l'encodeur droit depuis sa dernière remise à zéro
int16 ng=0;// le nombre de "tics" envoyés par l'encodeur gauche depuis sa dernière remise à zéro
float32 dd=0;// la distance parcourue par la roue droite depuis la dernière remise à zéro des encodeurs
float32 dg=0;// la distance parcourue par la roue gauche depuis la dernière remise à zéro des encodeurs
float32 d=0;// la distance parcourue par le centre du robot depuis la dernière remise à zéro des encodeurs
float32 a=0;// l'angle dont a tourné le robot depuis la dernière remise à zéro des encodeurs
float32 vitg=0;// La vitesse de la roue gauche
float32 vitd=0;// La vitesse de la roue droite
float32 teta_point; // La vitesse angulaire du robot


float32 v=0; // La vitesse le long de la ligne lorsqu'on avance en ligne droite
const float32 vmax=50; // La vitesse maximale lorsqu'on avance en ligne droite
const float32 vmin=30; // La vitesse minimale lorsqu'on avance en ligne droite
const float32 vmaxt=40; // La vitesse maximale lorsqu'on tourne
const float32 vmint=20; // La vitesse minimale lorsqu'on tourne

const float32 l=230;// la distance entre les roues du robot (milieu-milieu) en mm

const int16 puissancec_max=100;

const int8 periodepwm=100;


////////////////////////////////////////////////////////////////////////////////

// Fonctions utile...
float32 modulo_2_pi(float32 x)
{
    return x-floor(x/(PI*2.0))*2.0*PI;
}

// Fonction donnant la vitesse des roues pour l'asservissement. 
// _x correspont a la distance totale, _y a ce qui a ete parcouru.
float32 fonction_vitesse(float32 _x, float32 _y)
{
    return exp(-((_x-_y/2)/(_y/3))*((_x-_y/2)/(_y/3))*((_x-_y/2)/(_y/3))*((_x-_y/2)/(_y/3)));
}

int16 modulo_360(int16 x)
{
    //return x-floor((x/(360)))*360;
   return x%360;
}


////////////////////////////////////////////////////////////////////////////////

// La fonction qui analyse les données des encodeurs pour connaître les déplacements du robot.
// A appeler regulierement (toutes les 10 ms au moins)
void actualisepos()
{
    nd=-qei_get_count(1);// on relève la valeur du compteur de l'encodeur droit
    ng=qei_get_count(2);// on relève la valeur du compteur de l'encodeur gauche
    qei_set_count(1,0);// on remet l'encodeur droit à zéro
    qei_set_count(2,0);// on remet l'encodeur gauche à zéro
   dd=0.0005877349162882185*nd;
   dg=0.0005877349162882185*ng;
    d=(dd+dg)/2.0;// on calcule la distance parcourue par le centre du robot depuis la dernière actualisation
   a=1.0147582697201019*(dd-dg)/l;
   teta=teta-a*0.9554594594594594;

    // on fait en sorte que teta reste entre 0 et 2*PI
   /*
    if (teta>2*PI)
    {
        teta=teta-2*PI;
    }
    if (teta<0)
    {
        teta=teta+2*PI;
    }
   */
    float32 tet = modulo_2_pi(teta-a*0.044636363636363634);
   x=x+d*cos(tet);
   y=y+d*sin(tet);
   tetadeg=teta*57.2957795;// on calcule la valeur de teta en degres
}

////////////////////////////////////////////////////////////////////////////////


// Fonction qui gere au plus bas niveau les moteurs. 
// pg / pd puissance gauche / droit
void avance(float pg, float pd)
{
   // int8 normallement
    int8 g=floor(pg*periodepwm/100.0);
    int8 d=floor(pd*periodepwm/100.0);
    
    // Moteur gauche
    if (pg>0)
    {
      // On avance
      set_motor_unit( 1,2,MPWM_INDEPENDENT | MPWM_ENABLE_H | MPWM_FORCE_L_0,0,0);
    }
    else if (pg<0)
    {
      // On recule
      set_motor_unit( 1,2,MPWM_INDEPENDENT | MPWM_ENABLE_L | MPWM_FORCE_H_0,0,0);
    }
    else
    {
      // On utilise le frein moteur
      set_motor_unit( 1,2,MPWM_INDEPENDENT,0,0);
    }
    
   // Moteur droit
    if(pd>0)
    {
      // On avance
       set_motor_unit( 1,1,MPWM_INDEPENDENT | MPWM_ENABLE_L | MPWM_FORCE_H_0,0,0);
    }
    else if (pd<0)
    {
      // On recule 
       set_motor_unit( 1,1,MPWM_INDEPENDENT | MPWM_ENABLE_H | MPWM_FORCE_L_0,0,0);
    }
   else
   {
      // On utilise le frein moteur
      set_motor_unit( 1,1,MPWM_INDEPENDENT,0,0);
   }
   
   set_motor_pwm_duty(1,1,abs(d));//floor((511-pd*511/100))); // le moteur droit tourne avec un dc de 511 à 0
   set_motor_pwm_duty(1,2,abs(g));
}


////////////////////////////////////////////////////////////////////////////////

// Variables pour la fonction suivante

// Calcul de l'integrale de l'erreur
float32 interrg=0;
float32 interrd=0;

// Fonction qui sert a asservir lavitesse des roues du robot. S'appuie sur avance.
// temps et periode sont en us.
void asservit(float32 vcg, float32 vcd, int16 temps, int16 periode)
{
   // periode en microsecondes
    float32 vg=0;
    float32 vd=0;
    float32 pg=0;
    float32 pd=0;
    int16 tps=0;
    float32 kpv=1;
    float32 kdv=0.001;
    float32 kiv=10;
    float32 errprecg=0;
    float32 errprecd=0;
    float32 derrg=0;
    float32 derrd=0;
    int8 n=0;
    while (tps<temps)
    {
        actualisepos();
      // Les vitesses actuelles des roues
        vg=1000000*dg/periode;
        vd=1000000*dd/periode;
      
      // Pour le correcteur :

      // Calcul de la derivee
        derrg=1000000*((vcg-vg)-errprecg)/periode;
        derrd=1000000*((vcd-vd)-errprecd)/periode;
      // Calcul de l'erreur
        errprecg=vcg-vg;
        errprecd=vcd-vd;
      // Calcul de l'integrale
        interrg+=periode*errprecg/1000000;
        interrd+=periode*errprecd/1000000;
      // Asservissement
        pg=MAX(-99,MIN(99,floor(kpv*(vcg-vg)+kiv*interrg+kdv*derrg)));
        pd=MAX(-99,MIN(99,floor(kpv*(vcd-vd)+kiv*interrd+kdv*derrd)));
        
      avance(pg, pd);
      
        delay_us(periode);
      // Un peu faux... mais bon c'est un limiteur de boucles
        tps+=periode;
        n+=1;
    }
    avance(0,0);
}   


////////////////////////////////////////////////////////////////////////////////




// Prend en entree un angle en radians entre 0 et PI. 
void tourner(float32 thetacdeg)
{
   // Recoit un angle en radians. 
   float32 theta_init=teta;
   // Convertion degres radians
   float32 thetac=thetacdeg*0.0174532925;
   float32 v;
   
   // La fonction actualisepos ne remet pas teta ni tetadeg dans les intervalles conventionnels. Cela simplifie les calculs.
   // Seule la fonction main modifie ces intervalles. 

   // On tourne a droite
   if(etat_robot == 3)
   {
      while (teta<theta_init+thetac)
      {
         // Fonction de pause
         while(etat_robot == 5)
         {
            avance(0,0);
            actualisepos();
         }
         // Si l'etat du robot est repasse en attente d'ordre (fonction delete).
         if(etat_robot == 0)
            return;
         
         actualisepos();
         v=vmint+(vmaxt-vmint)*fonction_vitesse(thetac, abs(teta-theta_init));
         asservit(-v,v,10000,10000);
      }
   }
   // On tourne a gauche
   else
   {
      while (teta>theta_init-thetac)
      {
         // Fonction de pause
         while(etat_robot == 5)
         {
            avance(0,0);
            actualisepos();
         }
         // Si l'etat du robot est repasse en attente d'ordre (fonction delete).
         if(etat_robot == 0)
            return;
         
         actualisepos();
         v=vmint+(vmaxt-vmint)*fonction_vitesse(thetac, abs(teta-theta_init));
         asservit(v,-v,10000,10000);
      }
   }
   asservit(0,0,500000,10000);
   interrg=0;
   interrd=0;
   etat_robot = 0;
}


////////////////////////////////////////////////////////////////////////////////
// Suit une ligne droite partant de la position du robot a la position de l'objectif
void suivreligne(int16 distancec)
{
   // Calcul des coordonnees finales
   float32 xc=x+(float32)(distancec)*cos(teta);
   float32 yc=y+(float32)(distancec)*sin(teta);

   // Initialisation des variables
    interrg=0;
    interrd=0;
    //set_motor_unit( 1,1,MPWM_INDEPENDENT | MPWM_ENABLE | MPWM_FORCE_H_0,0,0);
    //set_motor_unit( 1,2,MPWM_INDEPENDENT | MPWM_ENABLE | MPWM_FORCE_L_0,0,0);
    int16 pas=10000; 
    float32 xd=x;// on enregistre la coordonnée x de départ de notre ligne droite
    float32 yd=y;// on enregistre la coordonnée y de départ de notre ligne droite
    float32 longueur=sqrt(((xc-xd)*(xc-xd))+((yc-yd)*(yc-yd)));
    float32 vectx=(xc-xd)/longueur;// on créé la coordonnée x du vecteur normalisé directeur de notre ligne droite
    float32 vecty=(yc-yd)/longueur;// on créé la coordonnée y du vecteur normalisé directeur de notre ligne droite
    float32 vectortx=vecty;// on créé la coordonnée x du vecteur normalisé perpendiculaire à notre ligne droite
    float32 vectorty=-vectx;// on créé la coordonnée y du vecteur normalisé perpendiculaire à notre ligne droite
    float32 dist=0;//on créé la distance parcourue sur la ligne droite (elle correspond à la projection de (x,y) sur cette droite)
    float32 erreur=0;// on créé l'erreur de position du robot, c'est à dire la distance de (x,y) à la droite
    //float longueur=sqrt((xc-x)*(xc-x)+(yc-y)*(yc-y));// on calcule la longueur de la droite que l'on doit suivre
    float32 derreur=0; // derivee erreur
    float32 erreurprec=0; // erreur precedente
   // Coefficients PID
    float32 kd=5;
    float32 kp=10;
    float32 ki=0;
    float32 integerr=0;

	float32 r;
	float32 vd;
	float32 vg;

    integerr=1;
    v=vmin;
    integerr=0;
    while (dist<longueur && (etat_robot==1 || etat_robot == 2))// tant qu'on n'est pas arrivé
    {
         // Fonction de pause
         while(etat_robot == 5)
         {
            avance(0,0);
            actualisepos();
         }
         // Si l'etat du robot est repasse en attente d'ordre (fonction delete).
         if(etat_robot == 0)
            return;
         
        actualisepos();
        dist=sqrt(((x-xd)*(x-xd))+((y-yd)*(y-yd)));// on actualise la distance que l'on a parcourue sur la droite
        erreur=(x-xd)*vectortx+(y-yd)*vectorty;// on actualise l'erreur de position
        derreur=(erreur-erreurprec)/(0.000001*pas);
        // Deux lignes supprimees
        erreurprec=erreur;
        integerr+=erreur*pas/1000000.0;
        //v=vmin+(vmax-vmin)*exp(-((dist-longueur/2)/(longueur/3))*((dist-longueur/2)/(longueur/3))*((dist-longueur/2)/(longueur/3))*((dist-longueur/2)/(longueur/3)));
        v=vmin+(vmax-vmin)*fonction_vitesse(dist, longueur);
        //float vg=v*(1-kp*erreur-kd*derreur-ki*integerr);
        //float vd=v*(1+kp*erreur+kd*derreur+ki*integerr);
        //float r=kp*erreur+kd*derreur+ki*integerr;
        // A differencier pour avancer et reculer
        r=MAX(-5,MIN(l*0.5*(kp*erreur+kd*derreur),5));
        vg=v-r;
        vd=v+r;

      // On asservit en distance parcourue sur la droite.
      // On envoie une vitesse positive pour avancer, negative pour reculer.
      // Sachant que vg et vd sont positifs.
      if(etat_robot == 2)
          asservit(-vg,-vd,pas,10000);
      else
         asservit(vg,vd,pas,10000);
    }
    if (dist>=longueur)
    {
      etat_robot=0;
    }
   else
   {
      // On est en pause
      // etat_robot=2;
   }
    asservit(0,0,500000,10000);
    interrg=0;
    interrd=0;
   etat_robot = 0;
}


////////////////////////////////////////////////////////////////////////////////







// --------------------- Fonctions i2c ------------------------

/*
Etats du robot : 
0 : en attente d'ordre
1 : avancer
2 : reculer
3 : tourner droit
4 : tourner gauche
5 : mets de la puissance sur les moteurs
6 : en pause
*/

// Fonctions utiles pour le traitement de la communication.
int1 attente()
{
   if(etat_robot == 0)
      return 1;
   return 0;
}

int1 est_pause()
{
   if(etat_robot == 6)
      return 1;
   return 0;
}


void avance(int16 distance)
{
   // Change l'etat du robot et donne la distance de consigne
   // Prend en entree la distance a parcourir, en mm.
   if(attente()==0)
      return;
      
   etat_robot = 1;
   distancec = distance;
}

//int8 loggg=0;
void recule(int16 distance)
{
   // Change l'etat du robot et donne la distance de consigne
   // Prend en entree la distance a parcourir, en mm.
   if(attente()==0)
      return;
      
   etat_robot = 2;
   //loggg=2;
   distancec = distance;
}

void tourne(int16 angle)
{
   // Change l'etat du robot et donne l'angle de consigne
   // Prend en entree l'angle a tourner (entre 0 et 360 degres), en degre en tournant a droite.
   if(attente()==0)
      return;
      
   etat_robot = 3;
   anglec=modulo_360(angle);
   if(anglec > 180)
   {
      etat_robot = 4;
      anglec=360-anglec;
   }
}

// Ou tourne_roues....
void puissance_pi(int16 puiss)
{
   // Mise en place d'une puissance brute sur les moteurs.
   // On recoit un entier non signe. On met ensuite un offset pour passer dans l'intervalle voulu
   if(attente()==0)
      return;
   
   if(puiss > 2*puissancec_max)
      return;
   puissancec=puiss-puissancec_max;
   etat_robot=5;
}

void pause()
{
   if(attente()==0 || est_pause() == 1)
      return;

   // On retient l'ancien etat du robot
   ret_etat_robot = etat_robot;
   // Change l'etat du robot pour le mettre en pause
   etat_robot = 6;
   // La suite des actions est a programmer
}

void play()
{
   // Change l'etat du robot pour le mettre en marche (ie quitte pause)
   if(est_pause() == 1)
      return;
   etat_robot = ret_etat_robot;
   ret_etat_robot = 100;
   // On remet en place les données
   
}

void delete()
{
   // Supprime l'action en pause. Permet de revenir a l'etat d'attente d'ordres.
   if(est_pause() == 1)
      return;
   etat_robot = 0;
   ret_etat_robot = 100;
}

void shutdown()
{
   // Utilisation de sleep pour "eteindre" le dsPIC.
   etat_robot = 7;
   sleep(SLEEP_FULL);
}


// --------------------- Communication i2c ------------------------

// Variables

/*
0 : addresse reçue, 
1 a 0x7F : byte de données reçu, 1 etant le premier, 2 le second, ....
0x80 : demande de réponse, 
0x81 a 0xFF : reponses a envoyer. Si 0x81, alors un byte de donnee a deja ete envoye. S'incremente a chaque envoit, commme pour la reception.
*/

BYTE data[8]; // Les données reçues 
//BYTE write[10];

BYTE state = 1; // L'état du bus I2C(0 : addresse reçue, 1 : byte de données reçu, 0x80 : demande de réponse)

int16 buff_i2c;

#INT_SI2C
void si2c_interrupt() {
    state = i2c_isr_state();
    if(state<=0x08)
    {
      data[state] = i2c_read();
      
      // Fonctions "systeme"
      if(state==0x01 && data[0x01] >= 30)
      {
          if(data[0x01] == 30)
             pause();
          else if(data[0x01] == 31)
             play();
          else if(data[0x01] == 32)
             delete();
          else if(data[0x01] == 33)
             shutdown();
      }
      // Fonctions de mouvement
      else if(state==0x03 && data[0x01] >= 20)
      {
         buff_i2c = make16(data[0x03], data[0x02]);
         if(data[0x01] == 20)
            avance(buff_i2c);
         else if(data[0x01] == 21)
            recule(buff_i2c);
         else if(data[0x01] == 22)
            tourne(buff_i2c);
         else if(data[0x01] == 23)
            puissance_pi(buff_i2c);
      }
      // Fonction set
      else if(state>=0x06 && data[0x01]==0)
      {
         // Ne pas appeler cette fonction quand le robot est en mouvement !!!
         // Lorsque toutes les donnees ont ete recues, on traite
         // Pour les int16, les bits de poids faible ont ete donnes en premier
         
         // On n'a pas le temps de convertir en float32, sinon on a un connection time out. 
         // On calculera les coordonnees plus tard.
         set_donnees=1;
         xpi = make16(data[0x03], data[0x02]);
         ypi = make16(data[0x05], data[0x04]);
         tetadeg = make16(data[0x07], data[0x06]);
      }
    }
    else if(state>=0x80)
    {
      //i2c_write(data[state-0x80]);
      if(state < 0x82)
         i2c_write(0x00);
      else if(state == 0x82)
         i2c_write(etat_robot);
      else if(state == 0x83)
         i2c_write((int16)x);
      else if(state == 0x84)
         i2c_write(((int16)x)>>8);
      else if(state == 0x85)
         i2c_write((int16)y);
      else if(state == 0x86)
         i2c_write(((int16)y)>>8);
      else if(state == 0x87)
         i2c_write((int16)tetadeg);
      else if(state == 0x88)
         i2c_write(((int16)tetadeg)>>8);
      else
         i2c_write(0x00);
    }
}

void main() 
{    
    //On active les interuptions physiques du PIC
    enable_interrupts(INT_SI2C);
    enable_interrupts(INTR_GLOBAL);
    
    // Initialisation des moteurs
    setup_qei(1,QEI_MODE_X2);// on initialise le compteur de l'encodeur droit
    setup_qei(2,QEI_MODE_X2);// on initialise le compteur de l'encodeur gauche
    qei_set_count(1, 0);
    qei_set_count(2, 0);
    output_high(PIN_B3);// on enable les ponts en H
    output_high(PIN_A0);
    setup_motor_pwm(1,MPWM_FREE_RUN , 100);
    
    int8 i=0;
    for(i=0; i < 8; i++)
      data[i]=0;
    
    while (etat_robot < 7)
    {
      // Soit il faut remettre les angles comme demande par le PI
      if(set_donnees==1)
      {
         set_donnees=0;
         x=(float32)xpi;
         y=(float32)ypi;
         teta=tetadeg*0.01745329251994;
      }
      // Soit on remet les valeurs des angles entre 0 et 2*PI et entre 0 et 360 au cas ou
      else
      {
         teta = modulo_2_pi(teta);
         tetadeg = modulo_360(tetadeg);
      }
      
      // Un mouvement à faire? 
	 // Soit en attente d'ordre, soit en pause. 
     if(attente() == 1 || est_pause() == 1)
       {
       // Utilisation du frein moteur pour rester arrete.
         avance(0,0);
       }
     else if(etat_robot == 1)
      {
         if(distancec == 0)
          {
            etat_robot = 0;
            avance(0,0);
          }
        else
        {
            suivreligne(distancec);
         // Envoi des consignes
            //avance(50,50);
            //actualisepos();
         //delay_us(10);
         }
      }
     else if(etat_robot == 2)
     {
         if(distancec == 0)
       {
            etat_robot = 0;
         avance(0,0);
         }
      else
      {
         // Envoi des consignes
         suivreligne(distancec);
         //avance(-50,-50);
         //actualisepos();
      }
     }
      else if(etat_robot == 3 || etat_robot == 4)
      {
      if(anglec == 0)
       {
         etat_robot = 0;
         avance(0,0);
       }
      else
      {
         // Envoi des consignes
         tourner(anglec);
         /*if(etat_robot==3)
            avance(-50,50);
         else
            avance(50,-50);
         actualisepos();*/
      }
      }
      else if(etat_robot == 5)
     {
      avance(puissancec,puissancec);
      actualisepos();
     }
     // Inutile. Au pire on perd de l'energie. Une instruction = 200ns
      //delay_us(5);
    }
}


