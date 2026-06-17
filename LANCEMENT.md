# Lancement du projet

## Dépendances

- ESP-IDF installé
- Python 3 + pyserial (`pip install pyserial`)

## Flash

```bash
. ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/cu.usbserial-0001 flash
```

Adapter le port série si besoin (`ls /dev/cu.*`).

## Test

```bash
python3 -m venv venv && source venv/bin/activate && pip install pyserial
python3 test_stress.py     # 13 tests automatisés
python3 test_fonctionel.py # scénario complet avec modèle véhicule
```

Le port série doit être libre avant de lancer les scripts (pas de monitor actif).
