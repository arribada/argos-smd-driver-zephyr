# Utiliser un Firmware Externe pour les Tests OTA

Ce guide explique comment utiliser votre propre fichier firmware pour les tests OTA au lieu du firmware de test intégré.

## 🎯 Vue d'Ensemble

Par défaut, l'application de test OTA utilise un petit firmware de test intégré (~512 bytes). Vous pouvez spécifier votre propre fichier firmware binaire qui sera automatiquement converti en tableau C pendant la compilation.

## 📝 Étapes

### 1. Préparer Votre Fichier Firmware

Vous avez besoin d'un fichier binaire (`.bin`) contenant le firmware à flasher. Par exemple:
- `firmware_v1.2.3.bin`
- `argos_app.bin`
- Tout fichier binaire valide

### 2. Build avec Firmware Externe

#### Option A: Via Ligne de Commande

```bash
west build -b adafruit_feather_nrf52840 samples/ota_test -- \
  -DCONFIG_ARGOS_OTA_FIRMWARE_FILE="/chemin/vers/firmware.bin"
```

**Exemple complet:**
```bash
# Avec un firmware dans /tmp
west build -b adafruit_feather_nrf52840 samples/ota_test -- \
  -DCONFIG_ARGOS_OTA_FIRMWARE_FILE="/tmp/mon_firmware_v1.2.3.bin"

# Avec un firmware dans le projet
west build -b adafruit_feather_nrf52840 samples/ota_test -- \
  -DCONFIG_ARGOS_OTA_FIRMWARE_FILE="${PWD}/firmwares/argos_app.bin"
```

#### Option B: Via menuconfig

```bash
west build -b adafruit_feather_nrf52840 samples/ota_test -t menuconfig
```

Puis naviguez vers:
```
Argos SMD OTA Test Configuration
  → Path to firmware file for OTA testing
```

Entrez le chemin complet de votre fichier firmware.

#### Option C: Via prj.conf

Modifiez `samples/ota_test/prj.conf`:

```conf
# Chemin vers le firmware externe
CONFIG_ARGOS_OTA_FIRMWARE_FILE="/chemin/absolu/vers/firmware.bin"
```

Puis build normalement:
```bash
west build -b adafruit_feather_nrf52840 samples/ota_test
```

### 3. Flash et Test

```bash
west flash

# Les logs montreront:
# "Using EXTERNAL firmware file"
# "Firmware size: XXXX bytes"
```

## 🔧 Comment ça Fonctionne

### 1. Conversion Automatique

Pendant le build, le script `fw_to_array.py` convertit votre firmware binaire en tableau C:

**Entrée** (`firmware.bin`):
```
4B 4E 53 46 01 01 00 00 ... (binary data)
```

**Sortie** (`firmware_image.h`):
```c
static const uint8_t firmware_image[] = {
    0x4B, 0x4E, 0x53, 0x46, 0x01, 0x01, 0x00, 0x00,
    ...
};
#define FIRMWARE_IMAGE_SIZE 32768
```

### 2. Inclusion dans le Code

Le fichier `main.c` utilise la macro `USE_EXTERNAL_FIRMWARE`:

```c
#ifdef USE_EXTERNAL_FIRMWARE
    #include "firmware_image.h"
    #define TEST_FW_IMAGE firmware_image
    #define TEST_FW_SIZE  FIRMWARE_IMAGE_SIZE
#else
    /* Built-in test firmware */
    static const uint8_t test_firmware[] = { ... };
    #define TEST_FW_IMAGE test_firmware
    #define TEST_FW_SIZE  sizeof(test_firmware)
#endif
```

## 📊 Limitations et Considérations

### Taille du Firmware

**RAM Disponible:**
- nRF52840: 256 KB RAM
- Firmware chargé en RAM pour les tests

**Recommandations:**
- **< 100 KB**: OK pour tests rapides
- **100-200 KB**: Possible, surveiller la RAM
- **> 200 KB**: Risque de manque de RAM

Si votre firmware est trop gros, envisagez:
1. Charger depuis flash externe
2. Charger depuis SD card
3. Stream directement depuis UART/USB

### Format du Firmware

Le firmware doit être au format attendu par le bootloader Argos SMD:
- Magic number correct
- Version valide
- CRC32 correct
- Format binaire brut (pas de header supplémentaire)

## 🧪 Exemples d'Utilisation

### Exemple 1: Test avec Firmware Réel

```bash
# Build avec firmware de production
west build -b adafruit_feather_nrf52840 samples/ota_test -- \
  -DCONFIG_ARGOS_OTA_FIRMWARE_FILE="/releases/argos_v2.1.0.bin" \
  -DCONFIG_ARGOS_OTA_TEST_FULL_UPDATE=y

# Flash et test
west flash

# Vérifier dans les logs:
# - "Using EXTERNAL firmware file"
# - "Firmware size: XXXX bytes"
# - "Firmware CRC32: 0xXXXXXXXX"
```

### Exemple 2: Test de Régression

```bash
# Tester plusieurs versions de firmware
for fw in v1.0.0.bin v1.1.0.bin v2.0.0.bin; do
    echo "Testing $fw..."

    west build -b adafruit_feather_nrf52840 samples/ota_test \
      -p auto -- \
      -DCONFIG_ARGOS_OTA_FIRMWARE_FILE="/firmwares/$fw"

    west flash

    # Attendre les résultats des tests
    sleep 60
done
```

### Exemple 3: CI/CD Pipeline

```yaml
# .github/workflows/ota-test.yml
name: OTA Tests

on: [push, pull_request]

jobs:
  ota-test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2

      - name: Setup Zephyr
        run: |
          # Setup Zephyr environment

      - name: Build with latest firmware
        run: |
          west build -b adafruit_feather_nrf52840 samples/ota_test -- \
            -DCONFIG_ARGOS_OTA_FIRMWARE_FILE="${GITHUB_WORKSPACE}/firmware/latest.bin"

      - name: Flash and Test
        run: |
          west flash
          # Wait for test completion
          # Parse test results
```

## 🔄 Régénération du Firmware Array

Le fichier `firmware_image.h` est régénéré automatiquement si:
- Le fichier firmware source change
- Le script `fw_to_array.py` change
- Build pristine (`west build -p pristine`)

Pas de régénération si:
- Seulement le code `main.c` change
- Configuration Kconfig change (mais pas le chemin firmware)

Pour forcer la régénération:
```bash
rm build/firmware_image.h
west build
```

## 📝 Script de Conversion Manuel

Vous pouvez aussi convertir manuellement:

```bash
# Convertir firmware en C array
python3 samples/ota_test/fw_to_array.py firmware.bin output.h

# Vérifier le résultat
head output.h
```

Output:
```c
/* Auto-generated from firmware.bin */
/* File size: 32768 bytes */

#ifndef FW_IMAGE_H
#define FW_IMAGE_H

#include <stdint.h>

static const uint8_t firmware_image[] = {
    0x4B, 0x4E, 0x53, 0x46, 0x01, 0x01, 0x00, 0x00,
    ...
};

#define FIRMWARE_IMAGE_SIZE 32768

#endif /* FW_IMAGE_H */
```

## ⚠️ Troubleshooting

### Erreur: "Firmware file not found"

```
CMake Error: Firmware file not found: /path/to/firmware.bin
```

**Solutions:**
1. Vérifier le chemin (doit être absolu)
2. Vérifier que le fichier existe: `ls -l /path/to/firmware.bin`
3. Vérifier les permissions: `chmod 644 /path/to/firmware.bin`

### Erreur: "No space left"

```
error: section '.rodata' will not fit in region 'FLASH'
```

**Cause:** Firmware trop gros pour la flash

**Solutions:**
1. Utiliser un firmware plus petit pour les tests
2. Augmenter la taille de la flash (si possible)
3. Charger depuis stockage externe

### Erreur: "Out of memory"

```
error: region 'RAM' overflowed
```

**Cause:** Firmware trop gros pour la RAM

**Solutions:**
1. Réduire la taille du firmware
2. Désactiver certains tests (`CONFIG_ARGOS_OTA_TEST_STRESS=n`)
3. Optimiser la configuration mémoire

## 📚 Fichiers Impliqués

| Fichier | Rôle |
|---------|------|
| `fw_to_array.py` | Script de conversion bin → C array |
| `CMakeLists.txt` | Orchestration de la conversion au build |
| `Kconfig` | Option `CONFIG_ARGOS_OTA_FIRMWARE_FILE` |
| `src/main.c` | Sélection firmware externe vs intégré |
| `build/firmware_image.h` | Fichier généré (ne pas commit!) |

## 💡 Astuces

### Vérifier le CRC32 du Firmware

```bash
# Calculer CRC32 du firmware
crc32 firmware.bin

# Comparer avec les logs du test
# "Firmware CRC32: 0xXXXXXXXX"
```

### Extraire la Version du Firmware

Si votre firmware a un header avec version:

```bash
# Afficher les premiers bytes
hexdump -C firmware.bin | head -n 5

# Exemple output:
# 00000000  4b 4e 53 46 01 02 00 00  ...
#           ^magic^    ^version^
```

### Build Multiple Firmware Tests

```bash
# Script pour tester plusieurs firmwares
#!/bin/bash

FW_DIR="/releases/firmwares"

for fw in $FW_DIR/*.bin; do
    echo "===== Testing $(basename $fw) ====="

    west build -b adafruit_feather_nrf52840 samples/ota_test \
      -p auto -- \
      -DCONFIG_ARGOS_OTA_FIRMWARE_FILE="$fw"

    if [ $? -eq 0 ]; then
        west flash
        echo "Press Enter when test complete..."
        read
    fi
done
```

## 🔗 Voir Aussi

- [README Principal](README.md) - Guide général des tests OTA
- [Pinout Feather](boards/FEATHER_PINOUT.md) - Configuration hardware
- [Documentation DFU](../../OTA_IMPLEMENTATION.md) - Détails implémentation

