#include <LiquidCrystal.h>
#include <Wire.h>
#include <DS3231.h>
#include <Servo.h>
#include <EEPROM.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

DS3231 RTC;
Servo servo;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *Stepper1 = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *Stepper2 = AFMS.getStepper(200, 2);

#define NUM_STATES 13

#define STATE_MAIN_TIME         0
#define STATE_CHANGE_CYCLES     1
#define STATE_LAMP_HEIGHT       2
#define STATE_MOD_LAMP_HEIGHT   3
#define STATE_TEMPERATURE       4
#define STATE_ETAT_POMPE        5
#define STATE_DERNIER_PH        6
#define STATE_ETAT_LAMPE        7
#define STATE_ASK_CHANGER_PH    8
#define STATE_CHANGER_PH        9
#define STATE_MESURER_PH        10
#define STATE_ADJUST_PH         11

#define JOYSTICK_UP     0
#define JOYSTICK_DOWN   1
#define JOYSTICK_LEFT   2
#define JOYSTICK_RIGHT  3
#define JOYSTICK_BUTTON 4
#define JOYSTICK_CENTER 5

#define EEPROM_DERNIER_PH       0
#define EEPROM_PH_VOULU         10
#define EEPROM_POMPE            20
#define EEPROM_PUMP_CYCLES      30

#define SensorPin 0
unsigned long int avgValue;
const float offset = 0.23; //calibrage de la sonde
int buf[10];
int temp;
float phValue;
float last_ph;
float pH_voulu;
float pH_offset = 1.00; //écart de pH accepté entre voulu et réel
int servo_delay = 500; //temps que le servo reste pour faire tomber une goute

int servo_center = 90;

const int joystick_pin_X = 3;
const int joystick_pin_Y = 2;
const int joystick_pin_button = 8;
const int relay_lampe = 11;
const int relay_pompe = 12;

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);

bool etat_lampe;
bool etat_pompe;

int lamp_start_hour = 7;
int lamp_stop_hour = 22;

int pump_cycles_per_day;
long pump_start_time;
long pump_interval;
long pump_stop_time;
long pump_last_time;
long pump_delay = 70; //nombre secondes que la pompe reste alumée

int current_state;
int joystick_current_pos;

int joystick_pos();
void execute_state(int state);
int joystick_last_pos;

//pour RTC
bool Century = false;
bool h12;
bool PM;
byte ADay, AHour, AMinute, ASecond, ABits;
bool ADy, A12h, Apm;
int Day;

byte year;
byte month;
byte date;
byte DoW;
byte hour;
byte minute;
byte second;

long seconds_in_week = 7L*24L*60L*60L;
long time_now;

/*
Ce tableau (qui sera appelé "le tableau" par la suite) sert à savoir dans quel case de menu suivant aller quand on se situe dans une case de menu et qu'on bouge le joystick

par exemple: on se situe dans la case de menu STATE_MAIN_TIME (première colonne) et on pousse le joystick vers le bas (deuxième rangée)
le tableau nous rends la valeur STATE_ETAT_POMPE, qui est la case de menu vers lequel on veut aller dans ce cas.

Dans next_state_array[5][NUM_STATES], le [5] signifie qu'il y a 5 actions possibles (nombre de rangées) et
[NUM_SATES] est un entier défini avant qui indique le nombre de cases de menu existants (nombre de colonnes).
*/
int next_state_array[5][NUM_STATES] =
{
    // STATE
    // MAIN_TIME                CHANGE_CYCLES           LAMP_HEIGHT             MOD_LAMP_HEIGHT         TEMPERATURE         ETAT_POMPE          DERNIER_PH          ETAT_LAMPE          ASK_CHANGER_PH          CHANGER_PH          MESURER_PH              ADJUST_PH
    {   STATE_MAIN_TIME,        STATE_CHANGE_CYCLES,    STATE_LAMP_HEIGHT,      STATE_MOD_LAMP_HEIGHT,  STATE_MAIN_TIME,    STATE_MAIN_TIME,    STATE_MAIN_TIME,    STATE_MAIN_TIME,    STATE_DERNIER_PH,       STATE_CHANGER_PH,   STATE_DERNIER_PH,       STATE_ADJUST_PH     },  // haut
    {   STATE_ETAT_POMPE,       STATE_CHANGE_CYCLES,    STATE_ETAT_POMPE,       STATE_MOD_LAMP_HEIGHT,  STATE_TEMPERATURE,  STATE_ETAT_POMPE,   STATE_MESURER_PH,   STATE_ETAT_LAMPE,   STATE_ASK_CHANGER_PH,   STATE_CHANGER_PH,   STATE_MESURER_PH,       STATE_ADJUST_PH     },  // bas
    {   STATE_CHANGE_CYCLES,    STATE_CHANGE_CYCLES,    STATE_MAIN_TIME,        STATE_MOD_LAMP_HEIGHT,  STATE_TEMPERATURE,  STATE_TEMPERATURE,  STATE_ETAT_POMPE,   STATE_DERNIER_PH,   STATE_MESURER_PH,       STATE_CHANGER_PH,   STATE_MESURER_PH,       STATE_ADJUST_PH     },  // gauche
    {   STATE_LAMP_HEIGHT,      STATE_MAIN_TIME,        STATE_LAMP_HEIGHT,      STATE_MOD_LAMP_HEIGHT,  STATE_ETAT_POMPE,   STATE_DERNIER_PH,   STATE_ETAT_LAMPE,   STATE_ETAT_LAMPE,   STATE_ASK_CHANGER_PH,   STATE_CHANGER_PH,   STATE_ASK_CHANGER_PH,   STATE_ADJUST_PH     },  // droite
    {   STATE_MAIN_TIME,        STATE_CHANGE_CYCLES,    STATE_MOD_LAMP_HEIGHT,  STATE_LAMP_HEIGHT,      STATE_TEMPERATURE,  STATE_ETAT_POMPE,   STATE_MESURER_PH,   STATE_ETAT_LAMPE,   STATE_CHANGER_PH,       STATE_ADJUST_PH,    STATE_ADJUST_PH,        STATE_MESURER_PH    }   // bouton
};

void setup()
{
    //initialisation
    Serial.begin(9600);
    Wire.begin();
    lcd.begin(16, 2);
    servo.attach(10); //servo 1 sur motorshield
    AFMS.begin(); // Adafruit MotorShield v2

    servo.write(servo_center); //center le servo
    
    Stepper1->setSpeed(1000); //vitesse des moteurs pas-à-pas
    Stepper2->setSpeed(1000);
    Stepper1->release();
    Stepper2->release();

    pump_start_time = EEPROMReadlong(EEPROM_POMPE);

    EEPROM.get(EEPROM_PH_VOULU, pH_voulu);
    EEPROM.get(EEPROM_DERNIER_PH, last_ph);
    pump_cycles_per_day = EEPROM.read(EEPROM_PUMP_CYCLES);

    pump_interval = (24L*60L*60L)/pump_cycles_per_day; //secondes entre chaque mise en marche de la pompe

    current_state = STATE_MAIN_TIME; //état dans lequel on commence
    execute_state(STATE_MAIN_TIME); //entrer dans cet état

    joystick_last_pos = joystick_pos();

    pinMode(joystick_pin_button, INPUT);
    pinMode(relay_lampe, OUTPUT);
    pinMode(relay_pompe, OUTPUT);
    digitalWrite(relay_pompe, HIGH);
}


void loop()
{
    RTC.getTime(year, month, date, DoW, hour, minute, second);
    time_now = SoW(DoW, hour, minute, second);
    manage_lamp();
    manage_pump();
    pH_value();

    if (EEPROMReadlong(EEPROM_POMPE) != pump_start_time) //sauvegarder l'heure de la prochaine pompe
    {
        EEPROMWritelong(EEPROM_POMPE, pump_start_time);
    }

    /*
    Les fonctions if qui suivent servent à afficher des choses à l'écran qui
    changent régulièrement (comme l'heure, le pH actuel etc.).
    */
    if (current_state == STATE_MAIN_TIME) //affiche l'heure en temps réel
    {
        lcd.setCursor(0, 0);
        print_time();
        lcd.setCursor(0, 1);
        print_date();
    }
    
    if (current_state == STATE_CHANGE_CYCLES) //Menu intéractif qui permet de changer la valeur du pH voulu
    {
        lcd.setCursor(12, 1);
        lcd.print(pump_cycles_per_day);

        if (joystick_pos() == JOYSTICK_UP)
        {
            pump_cycles_per_day += 1;
            EEPROM.update(EEPROM_PUMP_CYCLES, pump_cycles_per_day); //sauvegarder

            pump_interval = (24L*60L*60L)/pump_cycles_per_day;
            pump_start_time = (time_now  + pump_interval)%seconds_in_week;
            pump_stop_time = (time_now + pump_delay)%seconds_in_week;

        }

        else if (joystick_pos() == JOYSTICK_DOWN)
        {
            pump_cycles_per_day -= 1;
            EEPROM.update(EEPROM_PUMP_CYCLES, pump_cycles_per_day);

            pump_interval = (24L*60L*60L)/pump_cycles_per_day;
            pump_start_time = (time_now  + pump_interval)%seconds_in_week;
            pump_stop_time = (time_now + pump_delay)%seconds_in_week;
        }
    }

    if (current_state == STATE_TEMPERATURE) //affiche température
    {
        lcd.setCursor(0, 1);
        lcd.print(RTC.getTemperature());
    }

    if (current_state == STATE_CHANGER_PH) //Menu intéractif qui permet de changer la valeure du pH voulu
    {
        lcd.setCursor(10, 1);
        lcd.print(pH_voulu);

        if (joystick_pos() == JOYSTICK_UP)
        {
            pH_voulu = pH_voulu + 0.5;
            EEPROM.put(EEPROM_PH_VOULU, pH_voulu);
            
            
        }

        else if (joystick_pos() == JOYSTICK_DOWN)
        {
            pH_voulu = pH_voulu - 0.5;
            EEPROM.put(EEPROM_PH_VOULU, pH_voulu);
        }
    }

    if (current_state == STATE_MOD_LAMP_HEIGHT) //controller les moteurs pas-à-pas avec le joystick
    {
        while (joystick_pos() == JOYSTICK_UP)
        {
            Stepper1->step(1, FORWARD, DOUBLE);
            Stepper2->step(1, FORWARD, DOUBLE);
        }

        while (joystick_pos() == JOYSTICK_DOWN)
        {
            Stepper1->step(1, BACKWARD, DOUBLE);
            Stepper2->step(1, BACKWARD, DOUBLE);
        }
    }
    else
    {
        Stepper1->release();
        Stepper2->release();
    }

    if (current_state == STATE_ETAT_POMPE) //affiche la prochaine où la pompe se mettra en marche ou dans combien de temps elle s'arretera
    {
        if (etat_pompe == 1)
            {
                int seconds_left = (pump_stop_time - time_now)%60;
                int minutes_left = (pump_stop_time - time_now)/60;
                lcd.setCursor(0, 0);
                lcd.print("Pompe en marche.");

                lcd.setCursor(0, 1);
                lcd.print("0");
                lcd.print(minutes_left);
                lcd.print(":");
                if (seconds_left < 10)
                {
                    lcd.print("0");
                }
                lcd.print(seconds_left);
            }
    }

    if (current_state == STATE_MESURER_PH) //affiche et actualise le pH actuel mesuré par la sonde et permet de l'enregistrer en poussant sur le bouton du joystick
    {
        lcd.setCursor(0, 1);
        lcd.print("pH:");
        lcd.print(phValue);
        if (joystick_pos() == JOYSTICK_BUTTON)
        {
            last_ph = phValue;
            EEPROM.put(EEPROM_DERNIER_PH, last_ph);
        }
    }

    /*
    Ce qui suit sert à controller le menu et à passer d'une case de menu à une autre.
    */

    joystick_current_pos = joystick_pos();

    if (joystick_current_pos == joystick_last_pos) //si le position actuelle du joystick est la même que la position précédente, on ne fait rien
    {
        return; //on arrete tout et on recommence au début de la boucle, tout ce qui ce trouve plus bas que cette ligne ne sera pas executé
    }

    /*
    Si on est arrivé jusqu'ici, c'est que la position du joystick à changé.
    Dans ce cas on remet la dernière valeure du joystick égale à la valeure actuelle.
    Ceci permet d'executer les actions que une seule fois quand le joystick change de position
    et permet de naviguer facilement dans le menu entier une case de menu à la fois.
    */

    joystick_last_pos = joystick_current_pos; 

    if( joystick_last_pos == JOYSTICK_CENTER) //si le joystick est centré, il ne se passe rien.
    {
        return;
    }

    /*
    Partie essentielle au fonctionnement du menu qui utilise le tableau vu ci dessus pour passer à la case de menu menu suivante.
    Le premier paramètre [joystick_last_pos] prends la dernière position du joystick et sert à choisir la rangée dans le tableau.
    Le deuxième paramètre [current_state] sert à dire dans quelle case de menu on se situe actuellement et selectionne la colonne dans le tableau.
    */
    
    int next_state = next_state_array[joystick_last_pos][current_state]; //on regarde dans le tableau quel est l'état vers lequel on va (next_state)

    if (current_state != next_state) // si on a changé de case de menu, on exécute la fonction qui permet d'afficher le contenu de cette nouvelle case de menu (voir ci dessous).
    {
        current_state = next_state;
        execute_state(current_state);
    }
}

/*
Fonction qui utilise le principe de fsm pour afficher ce qu'il faut dans chaque écran de menu.
*/

void execute_state(int state)
{
    switch(state)
    {
        case STATE_MAIN_TIME: //affiche l'heure
            lcd.clear();
            lcd.setCursor(0, 0);
            print_time();
            lcd.setCursor(0, 1);
            print_date();
            lcd.setCursor(15, 1);
            lcd.print("\x7E");
            break;
        
        case STATE_CHANGE_CYCLES:
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Arrosages/");
            lcd.setCursor(0, 1);
            lcd.print("jour");
            lcd.setCursor(15, 0);
            lcd.print("+");
            lcd.setCursor(15, 1);
            lcd.print( "-");
            break;
        
        case STATE_LAMP_HEIGHT:
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Lever/baisser");
            lcd.setCursor(0, 1);
            lcd.print("\x7F");
            lcd.setCursor(2, 1);
            lcd.print("la lampe.");
            lcd.setCursor(14,1);
            lcd.print("OK");
            break;
        
        case STATE_MOD_LAMP_HEIGHT:
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Mod. hauteur de");
            lcd.setCursor(0, 1);
            lcd.print("lampe en cours!");
            break;

        case STATE_TEMPERATURE: //affiche la temperature
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Temperature:");
            lcd.setCursor(0, 1);
            lcd.print(RTC.getTemperature());
            lcd.setCursor(5, 1);
            lcd.print("\xDF"); // °
            lcd.print("C");
            lcd.setCursor(15, 1);
            lcd.print("\x7E");
            break;

        case STATE_ETAT_POMPE: //affiche si la pompe est allumée ou l'heure du prochain allumage
            lcd.clear();
            lcd.setCursor(0, 1);
            lcd.print("\x7F");
            lcd.setCursor(15, 1);
            lcd.print("\x7E");

            if (etat_pompe == 0)
            {
                int pump_DoW = ((pump_start_time/(24L*60L*60L))%7)+1;
                int pump_hour = (pump_start_time/3600)%24;
                int pump_minute = (pump_start_time/60)%60;
                int pump_second = pump_start_time%60;

                lcd.setCursor(0, 0);
                lcd.print("Proch. pompe:");
                lcd.setCursor(2, 1);

                if (pump_hour < 10)
                {
                    lcd.print("0");
                }
                lcd.print(pump_hour);

                lcd.print(":");

                if (pump_minute < 10)
                {
                    lcd.print("0");
                }
                lcd.print(pump_minute);

                lcd.print(":");

                if (pump_second < 10)
                {
                    lcd.print("0");
                }
                lcd.print(pump_second);

                lcd.print("  ");
                day_name(pump_DoW);
            }
            break;

        case STATE_DERNIER_PH:  //affiche le dernier pH mesuré
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Dernier pH:");
            lcd.setCursor(0, 1);
            lcd.print("\x7F");
            lcd.setCursor(2, 1);
            lcd.print(last_ph);
            lcd.setCursor(15, 1);
            lcd.print("\x7E");
            break;

        case STATE_ETAT_LAMPE: //affiche si la lampe est allumée ou éteinte
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Lumiere est");
            lcd.setCursor(0, 1);
            lcd.print("\x7F");

            if (etat_lampe == 1)
            {
                lcd.setCursor(2, 1);
                lcd.print("ON");
            }

            else
            {
                lcd.setCursor(2, 1);
                lcd.print("OFF");
            }
            break;

        case STATE_ASK_CHANGER_PH: //demande si on veut modifier le pH
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Changer le pH");
            lcd.setCursor(0, 1);
            lcd.print("(");
            lcd.print(pH_voulu);
            lcd.print(")");
            lcd.setCursor(14, 1);
            lcd.print("OK");
            break;

        case STATE_CHANGER_PH: //permet de modifier la valeure du pH voulue
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Nouv. pH voulu +");
            lcd.setCursor(15, 1);
            lcd.print( "-");
            break;

        case STATE_MESURER_PH: //permet de mesurer le pH
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Mesurer le pH");
            lcd.setCursor(14, 1);
            lcd.print("OK");
            break;

        case STATE_ADJUST_PH:
            adjust_pH();
            break;
    }
}

void day_name(int DoW) //ecrire le nom des jours au lieu des chiffres
{
    switch (DoW)
    {
        case 1:
            lcd.print("Lu");
            break;

        case 2:
            lcd.print("Ma");
            break;

        case 3:
            lcd.print("Me");
            break;

        case 4:
            lcd.print("Je");
            break;

        case 5:
            lcd.print("Ve");
            break;

        case 6:
            lcd.print("Sa");
            break;

        case 7:
            lcd.print("Di");
            break;
    }
}

void manage_lamp() //allume et éteinds la lampe aux heures voulues
{
    if (hour >= lamp_start_hour && hour < lamp_stop_hour)
    {
        etat_lampe = 1;
        digitalWrite(relay_lampe, LOW);
    }
    else
    {
        etat_lampe = 0;
        digitalWrite(relay_lampe, HIGH);
    }
}

void manage_pump() //allume et éteinds la pompe quand il faut
{
    Serial.print("time_now:");
    Serial.println(time_now);

    Serial.print("pump_last_time:");
    Serial.println(pump_last_time);

    Serial.print("pump_start_time:");
    Serial.println(pump_start_time);

    Serial.print("pump_stop_time:");
    Serial.println(pump_stop_time);

    Serial.println();

    pump_start_time = EEPROMReadlong(EEPROM_POMPE);

    if (pump_last_time < pump_start_time) //en temps normal
    {
        if (time_now < pump_last_time || time_now > pump_start_time)
        {
            digitalWrite(relay_pompe, LOW);
            etat_pompe = 1;
            pump_last_time = time_now;
            pump_start_time = (time_now  + pump_interval)%seconds_in_week;
            pump_stop_time = (time_now + pump_delay)%seconds_in_week;
        }
    }

    else //quand il est dimanche soir et que la pompe ne doit pas se mettre en marche avant lundi matin
    {
        if (pump_start_time < time_now < pump_last_time)
        {
            digitalWrite(relay_pompe, LOW);
            etat_pompe = 1;
            pump_last_time = time_now;
            pump_start_time = (time_now  + pump_interval)%seconds_in_week;
            pump_stop_time = (time_now + pump_delay)%seconds_in_week;
        }
    }

    if (time_now >= pump_stop_time)
    {
        digitalWrite(relay_pompe, HIGH);
        etat_pompe = 0;
    }
}

void adjust_pH() //automatise le pH avec le servo
{
    if (last_ph > pH_voulu + pH_offset)
    {
        servo.write(0);
        delay(servo_delay);
        servo.write(servo_center);

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("pH-down ajoute!");
        lcd.setCursor(0, 1);
        lcd.print("Melangez puis ok");
    }

    else if (last_ph < pH_voulu - pH_offset)
    {
        servo.write(180);
        delay(servo_delay);
        servo.write(servo_center);

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("pH-up ajoute!");
        lcd.setCursor(0, 1);
        lcd.print("Melangez puis OK");
    }

    else
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("pH ok");
        lcd.setCursor(0, 1);
        lcd.print("OK pour cont.");
    }
}

int print_date() // affiche la date sur l'ecran lcd
{
    lcd.print("20");
    lcd.print(year);
    lcd.print("-");
    if (month < 10)
    {
        lcd.print("0");
    }
    lcd.print(month);
    lcd.print("-");
    if (date < 10)
    {
        lcd.print("0");
    }
    lcd.print(date);

    lcd.print("  ");
    day_name(DoW);
}

int print_time() //affiche l'heure sur l'ecran lcd.
{
    if (hour < 10)
    {
        lcd.print("0");
    }
    lcd.print(hour);
    lcd.print(":");

    if (minute < 10)
    {
        lcd.print("0");
    }
    lcd.print(minute);
    lcd.print(":");

    if (second < 10)
    {
        lcd.print("0");
    }
    lcd.print(second);
}

/*
La fonction suivante nous donne la position du joystick pour qu'on n'ait pas à
se soucier des valeurs que le joystick nous renvoie, mais seulement dans quel
position il se trouve.
*/

int joystick_pos()
{
    int joystick_X = analogRead(joystick_pin_X) / 32;
    int joystick_Y = analogRead(joystick_pin_Y) / 32;
    int joystick_button = digitalRead(joystick_pin_button);

    if (joystick_X > 29)
    {
        return JOYSTICK_RIGHT;
    }

    else if (joystick_Y > 29)
    {
        return JOYSTICK_DOWN;
    }

    else if (joystick_button == 0)
    {
        return JOYSTICK_BUTTON;
    }

    else if (joystick_Y < 5)
    {
        return JOYSTICK_UP;
    }

    else if (joystick_X < 5)
    {
        return JOYSTICK_LEFT;
    }
    else
    {
        return JOYSTICK_CENTER;
    }
}


long SoW(int DoW, int hour, int minute, int second) //Secondes dans la semaine
{
    long T = (DoW-1)*24;
    T += hour;
    T *= 60;
    T += minute;
    T *= 60;
    T += second;

    return T;
}

/*
Cette fonction à été copiée du code qui se trouve sur eole pour calibrer la sonde pH.
*/

void pH_value()
{
    for(int i = 0; i < 10; i++) // échantillonnage de 10 valeurs pour moyenner la mesure
    {
        buf[i] = analogRead(SensorPin);
        delay(10);
    }

    for(int i = 0; i < 9; i++) // triage des données récoltées dans le sens croissant
    {
        for(int j = i + 1; j < 10; j++)
        {
            if(buf[i] > buf[j])
            {
                temp = buf[i];
                buf[i] = buf[j];
                buf[j] = temp;
            }
        }
    }

    avgValue = 0;
    for(int i = 2; i < 8; i++) // calcul de la moyenne des mesures en ne prennant pas les deux plus petites ni les deux plus grandes
    {
        avgValue += buf[i];
    }

    avgValue = avgValue / 6; // car 6 mesures

    phValue = (float)avgValue * 5.0 / 1024; //convert the analog into volt
    phValue = 3.5 * phValue + offset;	//convert the millivolt into pH value
}

//Copy-pasted from http://playground.arduino.cc/Code/EEPROMReadWriteLong

//Created by Kevin Elsenberger
//June 2, 2013
//elsenberger.k at gmail.com

void EEPROMWritelong(int address, long value)
{
    //Decomposition from a long to 4 bytes by using bitshift.
    //One = Most significant -> Four = Least significant byte
    byte four = (value & 0xFF);
    byte three = ((value >> 8) & 0xFF);
    byte two = ((value >> 16) & 0xFF);
    byte one = ((value >> 24) & 0xFF);

    //Write the 4 bytes into the eeprom memory.
    EEPROM.write(address, four);
    EEPROM.write(address + 1, three);
    EEPROM.write(address + 2, two);
    EEPROM.write(address + 3, one);
}

long EEPROMReadlong(int address)
{
    //Read the 4 bytes from the eeprom memory.
    long four = EEPROM.read(address);
    long three = EEPROM.read(address + 1);
    long two = EEPROM.read(address + 2);
    long one = EEPROM.read(address + 3);

    //Return the recomposed long by using bitshift.
    return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}
