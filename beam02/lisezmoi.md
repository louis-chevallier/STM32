# Séparation de Source audio avec le STM

## Le but

Isoler une source audio dans un environnement bruité.
Amplifier une source lointaine en gardant le niveau de bruit à un bas niveau.

C'est la technique de la séparation de source et plus précisément du 'beamforming'.

On utilise un réseau de micro disposés selon un arrangement adapté.

En fait, les micros vont être alignés et espacés à intervalles connu et régulier ( par exemple, 4 micros espacés de 10cm).

Ce système est pointé vers la source à isoler.

Le traitement audio est basé sur l'idée du "delay and sum".

## Delay and sum

## Matériel

On veut échantillonner les 4 micros au moins à 20KHz.

