# Struttura Tesi #

## Introduzione ##
* Obiettivo della tesi e di cosa parla 
* Struttura della tesi
* Intrduzione al problema della saturazione nei controlli (focus on wind up effect)
## Modellazione ##
* Modellazione della saturazione 
  * cos'è la saturazione come si modella discorso deadzone e modello generale di un sistema con sat
* Modellazione del sistema di benchmark
  * modellazione del sistema massa molla, eventualmente introducendo incertezze, alludere identificazione
## Set up sperimentale ##
* Descrizione del set up sperimentale
  * Apparato sperimentale, come funziona ...
  * Descrizione dell'acquisizione dati e modalità di lavoro
* Identificazione del sistema
  * Descrizione del metodo(i) usato (enfasi su problema polo origine)
  * Risultati dell'identificazione del sistema
## Sintesi del controllore e dell'anti wind up ##
* Sintesi $H_{\infty}$ del PID + filtro
  * struttura del controllore
  * sintesi (con MATLAB)
* Sintesi dell'anti wind up
  * Struttura dell'anti wind up statico (diagamma blocchi) piena autorità
  * sintesi con le LMI
## Simulazione e prova sperimentale ##
* Risultati della simulazione
* Risultati dell'esperimento
* Considerazioni e differenze
## eventuale secondo sviluppo ##
## Conclusioni generali ##