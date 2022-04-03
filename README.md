# Decoder DCC accessori per scambi con Arduino, Adafruit PCA9685 e servo motori SG90
![License](https://img.shields.io/badge/License-MIT-green)

*English version [here](README.EN.md)*

## Indice
- [Introduzione](#introduzione)
- [Cosa serve](#cosa-serve)
- [Connessioni](#connessioni)
- [Configurazione](#configurazione)
- [Tabella delle CVs (configuration variables) utilizzate](#tabella-delle-cvs-configuration-variables-utilizzate)
- [Alcune specifiche sul funzionamento dello sketch](#alcune-specifiche-sul-funzionamento-dello-sketch)
- [Versione e contributors](#versione-e-contributors)

## Introduzione
Lo sketch per [Arduino Uno](https://store.arduino.cc/products/arduino-uno-rev3) di questo progetto implementa un completo decoder DCC di tipo accessori per azionare servo motori del tipo [SG90](https://google.com/search?q=servo+sg90) per muovere gli aghi degli scambi del vostro plastico ferroviario

![Overview](/images/DCC_Turnouts_Overview.jpg)

## Cosa Serve
Per realizzare un decoder scambi DCC completo occorrono:
* una scheda [Arduino Uno](https://store.arduino.cc/products/arduino-uno-rev3) o [compatibile](https://google.com/search?q=geekcreit+arduino+uno)
* un'interfaccia [Adafruit PCA9685](https://adafruit.com/product/815) (demo disponibile [qui](https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all)) capace di azionare contemporaneamente 16 servo; la PCA9685 utilizza il bus di comunicazione I2C di Arduino (quindi con soli 2 pin utilizzati) ed è possibile inserire più interfacce a cascata [ponticellando](https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all#chaining-drivers) alcuni pin dedicati allo scopo
* un'alimentatore esterno da 5V in CC, richiesta dall'interfaccia PCA9685, per azionare i servo
* uno shield o un'interfaccia che sia in grado di "leggere" il segnale DCC attraverso Arduino, io consiglio vivamente [lo shield o l'interfaccia](https://github.com/lucadentella/arduino-dccshield) dell'amico Luca Dentella

## Connessioni
* Arduino Uno o interfaccia compatibile
    - utilizzare il cavo USB (utile se si vuole monitorare il traffico sulla finestra del monitor seriale) o l'apposito spinotto di alimentazione a 5V
* Interfaccia / shield DCC
    - utilizzare l'apposita morsettiera DCC dell'interfaccia / shield alla quale collegherete il segnale proveniente dalla vostra centralina, il segnale DCC viaggia in corrente alternata e quindi potete collegare sia un senso che nell'altro
    - VCC va collegato ad uno dei pins di Arduino indicati con 5V (tensione)
    - GND va collegato ad uno dei pins di Arduino indicati con GND (massa)
    - DCC va collegato al pin dedicato all'ingresso dati con interrupt, nello sketch il pin dichiarato è il 2
    - ACK va collegato al pin dedicato al segnale di ACK, nello sketch il pin dichiarato è il 6

![Overview](/images/DCC_Turnouts_DCCInputToArduinoInterface.jpg)

* PCA9685
    - VCC va collegato ad uno dei pins di Arduino indicati con 5V (tensione)
    - GND va collegato ad uno dei pins di Arduino indicati con GND (massa)
    - SDA va collegato al pin A4
    - SCL va collegato al pin A5
    - utilizzare i pins PWM già colorati per agevolare la connessione con i vostri servo motori (giallo => PWM, rosso => VCC, nero => GND)
> **E' richiesta obbligatoriamente un'alimentazione esterna di 5V per comandare i servo motori, questo approccio evita il possibile sovraccarico di corrente su Arduino!**

![Overview](/images/DCC_Turnouts_PCA9685ConnectionDetail.jpg)

## Configurazione

Lo sketch prevede questa configurazione:
- definizione del numero dei servo gestiti, il valore di default è 2
- definizione dell'intervallo di movimento tra un ciclo e l'altro, il valore di default è 70 (in ms) in modo da realizzare un movimento lento e quanto più realistico degli aghi
- definizione dell'angolo / posizione iniziale e angolo / posizione finale mediante apposita configurazione
- definizione se il movimento deve essere effettuato da destra verso sinistra o da sinistra verso destra (modo inverso)

## Tabella delle CVs (configuration variables) utilizzate

| CV | Descrizione | Note | Valore di default |
|:--|:--|:--|--:|
| 33 | Intervallo tra un singolo movimento di posizione ed il successivo | ms | 70 |
| 34 | Posizione minima servo (high bits) | | 0 |
| 35 | Posizione minima servo (low bits) | | 0 |
| 36 | Posizione massima servo (high bits) | | 0 |
| 37 | Posizione massima servo (low bits) | | 50 |
| 38 - 53 | Modo inverso (38 = servo n. 1, 39 = servo n. 2, ecc..) - 0 = normale, qualsiasi valore diverso da zero = invertito | | 0 |
| 54 - 69 | Posizione del servo (54 = servo 1, 55 = servo 2, ecc..) - 0 = chiuso, qualsiasi valore diverso da zero = aperto | | 0 |

> Il range di valore ammesso per posizionare il servo va da 0 a 4096 ed è stato necessario utilizzare 2 distinte CV
> come da tabella sopra-riportata sia la posizione minima che massima sono indicate con high bits e low bits
> la formula utilizzata per definire la posizione esatta è **posizione = (high_bits * 256) + low_bits**

> La differenza tra posizione minima e posizione massima è di default impostata a 50
> in quanto per muovere gli aghi degli scambi occorre un movimento alquanto minimo

## Alcune specifiche sul funzionamento dello sketch

***
Struttura contenente lo stato di ogni servo:
- **moving** indica se il servo è in movimento oppure no
- **positionState** indica la posizione impostata
- **actualPosition** indica la posizione attuale in base alla quale viene deciso se deve essere eseguito lo spostamento verso il valore minimo o verso il valore massimo, ad ogni ciclo di loop dello sketch viene aumentato o diminuito di un'unità fino al raggiungimento della posizione desiderata; il ciclo di loop valuta lo stato ed il movimento di tutti i servi consentendo quindi di eseguire azioni simultanee su tutti i servo motori collegati
```c
typedef struct ServoStatus {
  bool moving;
  ServoPositionState positionState;
  long actualPosition;
};
```
***
Nel setup iniziale all'accensione di Arduino viene eseguito un ciclo per ogni servo dove viene: inizializzato il modo inverso, letta la posizione attuale e impostata la relativa posizione
```c
void setup() {
    ...
    for (int idx = 0; idx < NUM_SERVOS; idx++) {

        // Get servo states
        ServoPositionState positionState = (Dcc.getCV(OFFSET_POSITION_CV_VALUE + idx) != 0 ? THROWN : CLOSED);
        ServoReversedMode reversedMode = (Dcc.getCV(OFFSET_REVERSED_CV_VALUE + idx) != 0 ? ENABLED : DISABLED);

        // Init servo position & state
        servoStatus[idx].moving = false;
        servoStatus[idx].positionState = positionState;
        servoStatus[idx].actualPosition = getServoPosition(positionState, reversedMode);
        pwm.setPWM(idx, 0, servoStatus[idx].actualPosition);
    
    }
    ...
}
```
***
Ad ogni ciclo di loop dello sketch viene verificato se presente un comando DCC da processare tramite le opportune funzioni di callback previste dalla libreria. Questo approccio consente di elaborare i comandi in arrivo per consentire l'esecuzione del movimento opposto anche quando è ancora in esecuzione l'istruzione precedente
```c
void loop() {
    ...
    // Process DCC incoming packets
    Dcc.process();
    ...
}
```
***
Esecuzione del ciclo per eseguire contemporaneamente il movimento su ogni singolo servo fino al raggiungimento della posizione desiderata
```c
void loop() {
    ...
        // Loop for each servo
        for (int idx = 0; idx < NUM_SERVOS; idx++) {

            // Execute servo movement
            if (servoStatus[idx].moving) {

                // Get reversed mode & actual position
                ServoReversedMode reversedMode = (Dcc.getCV(OFFSET_REVERSED_CV_VALUE + idx) != 0 ? ENABLED : DISABLED);
                long actualPosition = servoStatus[idx].actualPosition;

                // Apply min or max based on position, reversed mode and movement type to be made
                if ((servoStatus[idx].positionState == THROWN && reversedMode == DISABLED) ||
                    (servoStatus[idx].positionState == CLOSED && reversedMode == ENABLED)) {
                    executeSingleMovementToMinValue(idx);
                } else {
                    executeSingleMovementToMaxValue(idx);
                }

                // Set moved to exec delay
                moved = true;
            }
        }

        // Apply delay between movement and next
        if (moved) {
            delay(servoDelay);
        }
    ...
}
```
***
Funzione che esegue il movimento del servo verso la posizione minima diminuendo di 1 unità il valore della posizione attuale e verificando che il movimento sia giunto al valore limite desiderato
> La funzione **executeSingleMovementToMaxValue** è identica a questa ma esegue il movimento verso la posizione massima
```c
void executeSingleMovementToMinValue(int idx) {
    // Get actual position
    long actualPosition = servoStatus[idx].actualPosition; 

    // Check if position is over min value
    if (actualPosition < servoMinValue) {
        actualPosition = servoMinValue;
    }

    // Check if there is an update to new servo position
    if (actualPosition > servoMinValue) {
        actualPosition--;
    }

    // Set new servo position
    pwm.setPWM(idx, 0, actualPosition);

    // Update last status values
    servoStatus[idx].actualPosition = actualPosition;
    servoStatus[idx].moving = (actualPosition != servoMinValue);
}
```
***
Funzione di callback del cambio posizione di uno scambio, viene verificato se il comando è diretto ad uno dei servi gestiti dallo sketch Controllando l'indirizzo principale assegnato al decoder in funzione del numero dei servi gestiti
> **Esempio:**
> Indirizzo assegnato al decoder: 10
> Numero di servo motori gestiti: 4
> Indirizzi validi: 10 => primo servo motore, 11 => secondo servo motore ... 13 => quarto servo motore
```c
void notifyDccAccTurnoutOutput(uint16_t Addr, uint8_t Direction, uint8_t OutputPower) {
    // Get decoder DCC address
    long dccAddress = Dcc.getAddr();

    // Check if DCC packets needs to be processed on one of the servos
    if (Addr >= dccAddress && Addr < dccAddress + NUM_SERVOS) {
  
        // Calculate no of servo
        int servoNum = Addr - dccAddress;
    ...
}
```
***

## Versione e Contributors
Versione: 1.0.0
Data di pubblicazione: Aprile 2022
Autore: [Giovanni Colasurdo](mailto:gio.colasurdo@gmail.com)