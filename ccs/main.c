/* 
 * File:   propre.c
 * Author: Antoine
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
#fuses NOALTI2C1  // I2C1 mapped to SDA1/SCL1 pins

////////////////////////////////////////////////////////////////////////////////
//USE
////////////////////////////////////////////////////////////////////////////////

#use delay(clock=10000000)//pour pouvoir manipuler le temps
#USE I2C(SLAVE, SCL=PIN_B8, SDA=PIN_B9,ADDRESS=0x42, FORCE_HW)

////////////////////////////////////////////////////////////////////////////////
//PINS
////////////////////////////////////////////////////////////////////////////////

#pin_select QEA1=PIN_B10 //on sélectionne les pins des encodeurs
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

//Tout se fait actuellement en cm. Il faudra convertir en mm.


float32 xbut=0; //la consigne de position que le raspberry nous envoie
float32 ybut=0;

int8 etat_robot=0;// vaut 0 si le robot n'a pas le droit de bouger (au départ et si il y a un obstacle), 1 si il peut bouger et 2 pour arrêter le programme (en cas d'arrêt d'urgence ou de fin des 90s))
int1 trajet_fini=0; // vaut 0 si le robot est en train de se déplacer vers xbut,ybut et vaut 1 si il est arrivé et attend une nouvelle consigne

float32 x=0;// l'abcisse du robot 
float32 y=0;// l'ordonnée du robot
float32 teta=0;// l'angle du robot
float32 tetadeg=0;// teta en degrés (plus pratique à lire qu'un angle en rad)

int8 nd=0;// le nombre de "tics" envoyés par l'encodeur droit depuis sa dernière remise à zéro
int8 ng=0;// le nombre de "tics" envoyés par l'encodeur gauche depuis sa dernière remise à zéro
float32 dd=0;// la distance parcourue par la roue droite depuis la dernière remise à zéro des encodeurs
float32 dg=0;// la distance parcourue par la roue gauche depuis la dernière remise à zéro des encodeurs
float32 d=0;// la distance parcourue par le centre du robot depuis la dernière remise à zéro des encodeurs
float32 a=0;// l'angle dont a tourné le robot depuis la dernière remise à zéro des encodeurs
float32 vitg=0;// La vitesse de la roue gauche
float32 vitd=0;// La vitesse de la roue droite
float32 teta_point; // La vitesse angulaire du robot

float32 l=23;// la distance entre les roues du robot (milieu-milieu) en cm

float32 v=0; // La vitesse le long de la ligne lorsqu'on avance en ligne droite
float32 vmax=20; // La vitesse maximale lorsqu'on avance en ligne droite
float32 vmin=10; // La vitesse minimale lorsqu'on avance en ligne droite
float32 vmaxt=7; // La vitesse maximale lorsqu'on avance en ligne droite

float leserreurs[500];
float lesdistances[500];
float lesvitessesg[500];
float lesvitessesd[500];
float lestheta[1000];

int8 periodepwm=100;
float32 pgprec=0;
float32 pdprec=0;

float32 interrg=0;
float32 interrd=0;

////////////////////////////////////////////////////////////////////////////////

void actualisepos()// la fonction qui analyse les données des encodeurs pour connaître les déplacements du robot
{
    nd=-qei_get_count(1);// on relève la valeur du compteur de l'encodeur droit
    ng=qei_get_count(2);// on relève la valeur du compteur de l'encodeur gauche
    qei_set_count(1,0);// on remet l'encodeur droit à zéro
    qei_set_count(2,0);// on remet l'encodeur gauche à zéro
    dd=(70/71.7)*nd*PI*3.5*49/(46*19456);// on calcule la distance parcourue par la roue droite depuis la dernière actualisation
    dg=(70/71.7)*ng*PI*3.5*49/(46*19456);// on calcule la distance parcourue par la roue gauche depuis la dernière actualisation
    d=(dd+dg)/2;// on calcule la distance parcourue par le centre du robot depuis la dernière actualisation
    a=0.997*(360/353.7)*(dd-dg)/l;// on calcule l'angle dont a tourné le robot depuis sa dernière actualisation
    teta=teta+a*0.982*180/185;// on actualise teta

    // on fait en sorte que teta reste entre 0 et 2*PI
    if (teta>2*PI)
    {
        teta=teta-2*PI;
    }
    if (teta<0)
    {
        teta=teta+2*PI;
    }
    x=x+d*cos(teta-a*0.982/22);// on actualise x
    y=y+d*sin(teta-a*0.982/22);// on actualise y
    tetadeg=teta*360/(2*PI);// on calcule la valeur de teta en degres
}

////////////////////////////////////////////////////////////////////////////////

// pg / pd puissance gauche / droit
void avance(float pg, float pd)
{
    int g=floor(pg*periodepwm/100);
    int d=floor(pd*periodepwm/100);
    if (((pg*pgprec)>0)*((pd*pdprec)>0))
    {
        set_motor_pwm_duty(1,1,abs(d));//floor((511-pd*511/100))); // le moteur droit tourne avec un dc de 511 à 0
        set_motor_pwm_duty(1,2,abs(g));//floor(pg*255/100)); //le moteur gauche tourne avec un dc de 0 à 255
    }
    else
    {
        if ((pg>0)*(pgprec<=0))
        {
            set_motor_unit( 1,2,MPWM_INDEPENDENT | MPWM_ENABLE | MPWM_FORCE_L_0,0,0);
        }
        if ((pg<=0)*(pgprec>=0))
        {
            set_motor_unit( 1,2,MPWM_INDEPENDENT | MPWM_ENABLE | MPWM_FORCE_H_0,0,0);
        }
        if ((pd>0)*(pdprec<=0))
        {
            set_motor_unit( 1,1,MPWM_INDEPENDENT | MPWM_ENABLE | MPWM_FORCE_H_0,0,0);
        }
        if ((pd<=0)*(pdprec>=0))
        {
            set_motor_unit( 1,1,MPWM_INDEPENDENT | MPWM_ENABLE | MPWM_FORCE_L_0,0,0);
        }
        set_motor_pwm_duty(1,1,abs(d));//floor((511-pd*511/100))); // le moteur droit tourne avec un dc de 511 à 0
        set_motor_pwm_duty(1,2,abs(g));
    }
    pgprec=pg;
    pdprec=pd;
}

////////////////////////////////////////////////////////////////////////////////

void asservit(float vcg, float vcd,long temps,long periode)
{
	// periode en microsecondes
    float vg=0;
    float vd=0;
    float pg=0;
    float pd=0;
    long tps=0;
    float kpv=1;
    float kdv=0.001;
    float kiv=10;
    float errprecg=0;
    float errprecd=0;
    float derrg=0;
    float derrd=0;
    int n=0;
    while (tps<temps)
    {
        actualisepos();
        vg=1000000*dg/periode;
        vd=1000000*dd/periode;
        derrg=1000000*((vcg-vg)-errprecg)/periode;
        derrd=1000000*((vcd-vd)-errprecd)/periode;
        errprecg=vcg-vg;
        errprecd=vcd-vd;
        interrg+=periode*errprecg/1000000;
        interrd+=periode*errprecd/1000000;
        pg=MIN(99,floor(kpv*(vcg-vg)+kiv*interrg+kdv*derrg));
        pd=MIN(99,floor(kpv*(vcd-vd)+kiv*interrd+kdv*derrd));
        avance(pg,pd);
        delay_us(periode);
        tps+=periode;
		
		// inutiles
        lesvitessesg[n]=vg;
        lesvitessesd[n]=vd;
        n+=1;
    }
    avance(0,0);
}   


////////////////////////////////////////////////////////////////////////////////
float modulo_2_pi(float x)
{
    return x-floor(x/(PI*2))*2*PI;
}


void tourner_gauche_3(float af,float vitesse_angulaire)
{
//float consigne_vitessegauche;
float consigne_vitessedroite;
float consigne_tetapoint;
float tetapoint=0;
float tetazero=teta;
float k=0;

float temps =0;

float dt=0.5; //temps en millisecondes
float vitesse_angulaire_finrampe=vitesse_angulaire/4;
float temps_finrampe=0.6;
float vitesse_angulaire_max;
 //float retard;

while (tetapoint<vitesse_angulaire_finrampe && teta-tetazero<af){
    actualisepos() ;//actualise x,y,vg,vd,teta,tetapoint
    k=k+1;
	temps=temps+dt;
    tetapoint=(vitd-vitg)/l;
    lestheta[k]=tetapoint;
	consigne_tetapoint=vitesse_angulaire_finrampe*(temps/temps_finrampe); // déterminons t0 ensemble
	consigne_vitessedroite=consigne_tetapoint/(2*l);
	//consigne_vitessegauche= -consigne_vitessedroite; Pour une raison inconnue ça ne marche pas avec cette variable...
	asservit(-consigne_vitessedroite,consigne_vitessedroite,dt*1000,10000);
}

//retard=teta-(af/PI)*asin(tetapoint*(4/PI));
vitesse_angulaire_max=tetapoint/sin(PI*(teta-tetazero)/af);

while(abs(teta-tetazero)<af){
	actualisepos() ;//actualise x,y,vg,vd,teta,tetapoint
	consigne_tetapoint=vitesse_angulaire_max*sin(PI*abs(teta-tetazero)/af);
	consigne_vitessedroite=consigne_tetapoint/(2*l);
    lestheta[k]=teta;
    tetapoint=(vitd-vitg)/l;
	//consigne_vitessegauche= -consigne_vitessedroite; Pour une raison inconnue ça ne marche pas avec cette variable...
	asservit(-consigne_vitessedroite,consigne_vitessedroite,dt*1000,10000);
	}
asservit(0,0,dt*1000,10000);
}

void tourner_droite_3(float af,float vitesse_angulaire)
{
float consigne_vitessegauche;
float consigne_vitessedroite;
float consigne_tetapoint;
float tetapoint=0;
float tetazero=teta;

float temps =0;
float temps_finrampe=0.6;

float dt=0.05;
float retard;

while (tetapoint<PI/8 && teta-tetazero<af){
actualisepos() ;//actualise x,y,vg,vd,teta,tetapoint
	temps=temps+dt;
    tetapoint=(vitd-vitg)/l;
	consigne_tetapoint=temps*(PI/(4*temps_finrampe*temps_finrampe)); // déterminons t0 ensemble
	consigne_vitessedroite=-consigne_tetapoint/(2*l);
	consigne_vitessegauche=-consigne_vitessedroite;
	asservit(vitg,vitd,dt,0.01);
}

retard=teta-(af/PI)*asin(tetapoint*(4/PI));

while(teta-tetazero<af){
	actualisepos() ;//actualise x,y,vg,vd,teta,tetapoint
	consigne_tetapoint=(PI/4)*sin(PI*(teta-retard)/af);
	consigne_vitessedroite=-consigne_tetapoint/(2*l);
	consigne_vitessegauche=-consigne_vitessedroite;
	asservit(vitg,vitd,dt,0.01);
	}
}


void tourner(float psi,float vitesse_angulaire){
          float af= modulo_2_pi(psi-modulo_2_pi(teta)); //angle a faire
          if (af<=PI)
                tourner_gauche_3(af,vitesse_angulaire);
                   
            else 
                tourner_droite_3(2*PI-af,vitesse_angulaire);
          asservit(0,0,500000,10000);
 }
                            
                        
    
    

void tourner(float tetac)
{
         
    if (teta<=PI)// on fait plein de disjonctions des cas pour savoir dans quel sens le robot doit tourner pour faire face à la bonne direction
    {
        if ((tetac>=teta)&&(tetac<=teta+PI))
        {
            while(teta<=tetac)
            {  
                actualisepos();
                asservit(-vmaxt,vmaxt,10000,10000);
            }
        }
        else
        {
            if (tetac<PI)
            {
               while (teta>tetac)
                {
                    actualisepos();
                    asservit(vmaxt,-vmaxt,10000,10000);
                } 
            }
            else
            {   
                while (teta<=PI)
                {
                    actualisepos();
                    asservit(vmaxt,-vmaxt,10000,10000);
                }
                actualisepos();
                asservit(vmaxt,-vmaxt,10000,10000);
                while (teta>=tetac)
                {
                    actualisepos();
                    asservit(vmaxt,-vmaxt,10000,10000);
                }
            }
        }
    }
    else 
    {
        if ((tetac>=teta-PI)&& (tetac<=teta))
        {
            while (teta>=tetac)
            {
               actualisepos();
               asservit(vmaxt,-vmaxt,10000,10000);
            }
        }
        else
        {
            if (tetac>PI)
            {
                while (teta<=tetac)
                {
                    actualisepos();
                    asservit(-vmaxt,vmaxt,10000,10000);
                }
            }
            else
            {
                while (teta>=PI)
                {
                    actualisepos();
                    asservit(-vmaxt,vmaxt,10000,10000);
                }
                actualisepos();
                asservit(-vmaxt,vmaxt,10000,10000);
                while (teta<=tetac)
                {
                    actualisepos();
                    asservit(-vmaxt,vmaxt,10000,10000);
                }
            }
        }
    }
    asservit(0,0,500000,10000);
}


float test=0;
////////////////////////////////////////////////////////////////////////////////
void suivreligne(float xc, float yc)
{
    etat_robot=1;
    interrg=0;
    interrd=0;
    set_motor_unit( 1,1,MPWM_INDEPENDENT | MPWM_ENABLE | MPWM_FORCE_H_0,0,0);
    set_motor_unit( 1,2,MPWM_INDEPENDENT | MPWM_ENABLE | MPWM_FORCE_L_0,0,0);
    int pas=10000; 
    float xd=x;// on enregistre la coordonnée x de départ de notre ligne droite
    float yd=y;// on enregistre la coordonnée y de départ de notre ligne droite
    float norm=sqrt(((xc-xd)*(xc-xd))+((yc-yd)*(yc-yd)));
    float vectx=(xc-xd)/norm;// on créé la coordonnée x du vecteur normalisé directeur de notre ligne droite
    float vecty=(yc-yd)/norm;// on créé la coordonnée y du vecteur normalisé directeur de notre ligne droite
    float vectortx=vecty;// on créé la coordonnée x du vecteur normalisé perpendiculaire à notre ligne droite
    float vectorty=-vectx;// on créé la coordonnée y du vecteur normalisé perpendiculaire à notre ligne droite
    float dist=0;//on créé la distance parcourue sur la ligne droite (elle correspond à la projection de (x,y) sur cette droite)
    float erreur=0;// on créé l'erreur de position du robot, c'est à dire la distance de (x,y) à la droite
    float longeur=sqrt((xc-x)*(xc-x)+(yc-y)*(yc-y));// on calcule la longeur de la droite que l'on doit suivre
    float derreur=0;
    float erreurprec=0; // Erreur precedente
    float kd=5;
    float kp=10;
    float ki=0;
    int u=0;
    float integerr=0;
    while (dist<longeur && etat_robot==1)// tant qu'on n'est pas arrivé
    {
        actualisepos();
        dist=sqrt(((x-xd)*vectx+(y-yd)*vecty)*((x-xd)*vectx+(y-yd)*vecty));// on actualise la distance que l'on a parcouru sur la droite
        erreur=(x-xd)*vectortx+(y-yd)*vectorty;// on actualise l'erreur de position. C'est un produit scalaire
        derreur=(erreur-erreurprec)/(0.000001*pas);
        erreurprec=erreur;
        integerr+=erreur*pas/1000000;
		// asservissement en vitesse
        v=vmin+(vmax-vmin)*exp(-((dist-longeur/2)/(longeur/3))*((dist-longeur/2)/(longeur/3))*((dist-longeur/2)/(longeur/3))*((dist-longeur/2)/(longeur/3)));
        //float vg=v*(1-kp*erreur-kd*derreur-ki*integerr);
        //float vd=v*(1+kp*erreur+kd*derreur+ki*integerr);
        //float r=kp*erreur+kd*derreur+ki*integerr;
        float r=max(-5,min((l/2)*(kp*erreur+kd*derreur),5));
        float vg=v-r;
        float vd=v+r;
        //avance(vg,vd,pas,periode);// on fournit la bonne puissance aux deux moteurs (asservissement de dist et erreur)
        asservit(vg,vd,pas,10000);

		// inutiles
        leserreurs[u]=erreur;
        lesdistances[u]=dist;
        u+=1;
    }
    if (dist>=longeur)
    {
        trajet_fini=1;
        etat_robot=0;
    }
    asservit(0,0,500000,10000);
    interrg=0;
    interrd=0;
}

////////////////////////////////////////////////////////////////////////////////

float compteur=0;

void aller(float xc,float yc)// la fonction qu'on appelle pour aller en (xc,yx), elle permet de gérer des exceptions comme xc=x qui empêchent de calculer l'angle tetac
{
    trajet_fini=0;
    if ((x-xc)*180/PI>-10 && (x-xc)*180/PI<10)// dans le cas ou xc est proche de x
    {
        if ((yc-y)<=0)// si on va vers le bas
        {
            tourner(3*PI/2);
            suivreligne(xc,yc);// tetac=3PI/2
        }
        else// si on va vers le haut 
        {
            tourner(PI/2);
            suivreligne(xc,yc);// tetac=PI/2
        }
    }
    else
    {
        if (x<xc)// si on va a droite
        {
            float tetac=atan((yc-y)/(xc-x));// on calcule tetac et on le normalise
            if (tetac>2*PI)
            {
                tetac-=2*PI;
            }
            if (tetac<0)
            {
                tetac+=2*PI;
            }  
            float diffangle=teta-tetac;
            if (diffangle>PI)
            {
                diffangle=diffangle-2*PI;
            }
            if (diffangle<-PI)
            {
                diffangle=diffangle+2*PI;
            }
            if (sqrt(diffangle*diffangle)>0.3)
            {
                compteur=diffangle;
                tourner(tetac);
            }
            suivreligne(xc,yc);
        }
        else// si on va a gauche
        {
            float tetac=PI+atan((yc-y)/(xc-x));// on calcule tetac et on le normalise
            if (tetac>2*PI)
            {
                tetac-=2*PI;
            }
            if (tetac<0)
            {
                tetac+=2*PI;
            }  
            float diffangle=teta-tetac;
            if (diffangle>PI)
            {
                diffangle=diffangle-2*PI;
            }
            if (diffangle<-PI)
            {
                diffangle=diffangle+2*PI;
            }
            if (sqrt(diffangle*diffangle)>0.3)
            {
                compteur=diffangle;
                tourner(tetac);
            }
            suivreligne(xc,yc);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

BYTE data = 0; // Les données reçues 
BYTE state = 1; // L'état du bus I2C(0 : addresse reçue, 1 : byte de données reçu, 0x80 : demande de réponse, 0x81 : demandede reponse apres avoir recu une donnee)


// Flag pour l'interruption du processeur lors d'un message arrivant via i2c dans le dsPIC.
//int nbr_bytes = 0;

#INT_SI2C
void si2c_interrupt() {
    state = i2c_isr_state();
    switch (state) {
        case 0:
            i2c_read();
            break;
        case 1:
            data = i2c_read();
            if(bytes_a_venir == 0 )
            {
                if (data<3)
                {
                    etat_robot=data;
                }
                else
                {
                    bytes_a_venir++;
                }
            }
            else if (bytes_a_venir==1)
            {
                xbut=300*data/255;
                bytes_a_venir++;
              
            }
            else if (bytes_a_venir==2)
            {
                ybut=data;
                bytes_a_venir=0;
            }
            break;
        case 0x80:
            i2c_write(trajet_fini);
            break;
        default:
            break;
    }
}

void main() 
{    
    enable_interrupts(INT_SI2C);
    enable_interrupts(INTR_GLOBAL);
    setup_qei(1,QEI_MODE_X2);// on initialise le compteur de l'encodeur droit
    setup_qei(2,QEI_MODE_X2);// on initialise le compteur de l'encodeur gauche
    output_high(PIN_B3);// on enable les ponts en H
    output_high(PIN_A0);
    setup_motor_pwm(1,MPWM_FREE_RUN , 100);
    setup_motor_pwm(2,MPWM_FREE_RUN , 100);    
    while (etat_robot<2)
    {
        if (etat_robot==1)
        {
            trajet_fini=0;
            aller(xbut,ybut);
        }
        else
        {
            actualisepos();
            delay_ms(1);
        }
    }
}
