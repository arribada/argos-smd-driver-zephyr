# Argos SMD OTA/DFU Implementation

## Vue d'ensemble

Cette implémentation ajoute le support complet de mise à jour firmware (OTA/DFU) via UART pour les modules Argos SMD. Le système supporte:

- ✅ Entrée en mode bootloader
- ✅ Transfert de firmware par chunks
- ✅ Vérification CRC32
- ✅ Gestion d'erreurs complète
- ✅ Tests automatisés

## Fichiers créés

### 1. API Publique DFU

#### `include/argos-smd/argos_dfu.h`
Header public exposant l'API DFU:
- `argos_enter_bootloader()` - Entrer en mode bootloader
- `argos_wait_bootloader_ready()` - Attendre le bootloader
- `argos_dfu_start()` - Démarrer session DFU
- `argos_dfu_send_chunk()` - Envoyer chunk de données
- `argos_dfu_finish()` - Finaliser la mise à jour
- `argos_dfu_abort()` - Annuler la session
- `argos_ota_update()` - Mise à jour complète (fonction de haut niveau)
- `argos_dfu_crc32()` - Calcul CRC32

### 2. Implémentation Driver

#### `drivers/argos-smd/argos_dfu.c`
Implémentation complète du driver DFU:
- Calcul CRC32 (polynomial 0xEDB88320)
- Conversion binaire vers hexadécimal
- Gestion des timeouts
- Log détaillé pour debugging
- Gestion d'erreurs robuste

#### `drivers/argos-smd/Kconfig`
Configuration Kconfig ajoutée:
```kconfig
CONFIG_ARGOS_DFU=y              # Activer support DFU
CONFIG_ARGOS_DFU_LOG_LEVEL      # Niveau de log (auto-configuré)
```

#### `drivers/argos-smd/CMakeLists.txt`
Build conditionnel:
```cmake
zephyr_library_sources_ifdef(CONFIG_ARGOS_DFU argos_dfu.c)
```

### 3. Application de Test

#### `samples/ota_test/`
Application complète de test avec:

##### `src/main.c`
Tests organisés en 6 phases:
1. **Tests basiques**: PING, VERSION
2. **Tests bootloader**: Entrée bootloader, PING bootloader
3. **Tests session DFU**: START, SEND_CHUNK, ABORT
4. **Tests d'erreur**: CRC invalide, chunk trop grand, DFU sans session
5. **Test OTA complet** (optionnel): Mise à jour complète
6. **Tests de stress** (optionnel): Mises à jour répétées

##### `prj.conf`
Configuration projet:
```
CONFIG_ARGOS_SMD=y
CONFIG_ARGOS_DFU=y
CONFIG_HEAP_MEM_POOL_SIZE=8192
CONFIG_MAIN_STACK_SIZE=4096
```

##### `Kconfig`
Options de configuration:
```
CONFIG_ARGOS_OTA_TEST_FULL_UPDATE=y  # Activer test OTA complet
CONFIG_ARGOS_OTA_TEST_STRESS=y       # Activer tests de stress
```

##### `CMakeLists.txt`
Configuration build standard

##### `README.md`
Documentation complète avec:
- Guide de build
- Configuration devicetree
- Référence des commandes AT
- Diagrammes de flux
- Exemples d'utilisation API
- Troubleshooting

##### `boards/nrf52840dk_nrf52840.overlay`
Exemple d'overlay devicetree pour nRF52840 DK

## Commandes AT Supportées

### Mode Application
- `AT+BOOT` - Entrer en bootloader
- `AT+VER=?` - Lire version
- `AT+FW=?` - Lire info firmware

### Mode Bootloader
- `AT+PING` - Tester connexion
- `AT+DFUSTART=<size>,<crc32>` - Démarrer DFU
- `AT+DFUDATA=<hex_data>` - Envoyer données (max 64 bytes binaires)
- `AT+DFUEND` - Finaliser DFU
- `AT+DFUABORT` - Annuler DFU
- `AT+DFUSTATUS=?` - Lire statut DFU

## Flow de Mise à Jour

```
Application Mode
      |
      | AT+BOOT
      v
   REBOOT
      |
      v
Bootloader Mode
      |
      | AT+PING (wait ready)
      v
  Bootloader Ready
      |
      | AT+DFUSTART=<size>,<crc>
      v
  DFU Session Active
      |
      | AT+DFUDATA=<hex> (repeat)
      v
  All Chunks Sent
      |
      | AT+DFUEND
      v
  Verify CRC & Flash
      |
      v
   REBOOT
      |
      v
Application Mode (New FW)
```

## Utilisation de l'API

### Exemple Simple

```c
#include <argos-smd/argos_dfu.h>

const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(argossmd));

/* Firmware en mémoire */
const uint8_t firmware[1024] = { /* ... */ };

/* Callback de progression (optionnel) */
void progress(uint32_t current, uint32_t total) {
    printk("Progress: %u%%\n", (current * 100) / total);
}

/* Mise à jour OTA complète */
int ret = argos_ota_update(dev, firmware, sizeof(firmware), progress);
if (ret == 0) {
    printk("OTA update successful!\n");
}
```

### Exemple Manuel (Contrôle Fin)

```c
/* Calculer CRC32 */
uint32_t crc = argos_dfu_crc32(firmware, fw_size);

/* Entrer en bootloader */
argos_enter_bootloader(dev);
argos_wait_bootloader_ready(dev, K_SECONDS(10));

/* Démarrer session */
argos_dfu_start(dev, fw_size, crc);

/* Envoyer chunks */
size_t offset = 0;
while (offset < fw_size) {
    size_t len = MIN(ARGOS_DFU_CHUNK_SIZE, fw_size - offset);
    argos_dfu_send_chunk(dev, &firmware[offset], len);
    offset += len;
}

/* Finaliser */
argos_dfu_finish(dev);
```

## Build et Test

### Build Standard
```bash
cd samples/ota_test
west build -b nrf52840dk_nrf52840
west flash
```

### Build avec Test OTA Complet
```bash
west build -b nrf52840dk_nrf52840 -- \
  -DCONFIG_ARGOS_OTA_TEST_FULL_UPDATE=y
```

⚠️ **ATTENTION**: Ceci effectuera une vraie mise à jour firmware!

### Build avec Tests de Stress
```bash
west build -b nrf52840dk_nrf52840 -- \
  -DCONFIG_ARGOS_OTA_TEST_STRESS=y
```

## Configuration Devicetree

```dts
&uart1 {
    status = "okay";
    current-speed = <9600>;

    argossmd {
        compatible = "arribada,argossmd";
        /* Optionnel: GPIO wakeup pour low power */
        /* wakeup-gpios = <&gpio0 15 GPIO_ACTIVE_HIGH>; */
    };
};
```

## Codes d'Erreur DFU

| Code | Nom | Description |
|------|-----|-------------|
| 0 | DFU_ERR_NONE | Pas d'erreur |
| 1 | DFU_ERR_INVALID_SIZE | Taille firmware invalide |
| 2 | DFU_ERR_INVALID_CRC | CRC32 ne correspond pas |
| 3 | DFU_ERR_FLASH_WRITE | Erreur écriture flash |
| 4 | DFU_ERR_FLASH_ERASE | Erreur effacement flash |
| 5 | DFU_ERR_NOT_STARTED | Session DFU non démarrée |
| 6 | DFU_ERR_OVERFLOW | Données dépassent taille annoncée |
| 7 | DFU_ERR_TIMEOUT | Timeout entre chunks |

## Tests Implémentés

### ✅ Tests Fonctionnels
- [x] PING en mode application
- [x] Lire version firmware actuelle
- [x] Entrer en mode bootloader
- [x] PING en mode bootloader
- [x] Démarrer session DFU
- [x] Envoyer chunks de données
- [x] Annuler DFU en cours
- [x] Finaliser DFU avec bon CRC
- [x] Vérifier nouvelle version après reboot

### ✅ Tests d'Erreur
- [x] DFU avec CRC invalide → rejeté
- [x] Chunk trop grand → rejeté
- [x] DFU sans session démarrée → rejeté
- [x] Double DFUSTART → session reset

### ✅ Tests de Stress (optionnels)
- [x] OTA de gros fichier (configurable)
- [x] OTA répétées (3x de suite)

## Checklist de Déploiement

### Pour Intégration
- [ ] Tester avec firmware réel du module Argos SMD
- [ ] Vérifier timeouts appropriés pour votre hardware
- [ ] Tester récupération après perte de connexion
- [ ] Valider CRC32 avec firmware officiel
- [ ] Tester avec différentes tailles de firmware
- [ ] Vérifier comportement en cas de batterie faible

### Pour Production
- [ ] Implémenter source de firmware (flash externe, SD card)
- [ ] Ajouter vérification de signature firmware
- [ ] Implémenter rollback en cas d'échec
- [ ] Ajouter logs persistants
- [ ] Tester update pendant opération normale
- [ ] Implémenter retry automatique avec backoff
- [ ] Ajouter watchdog pour timeout global

## Caractéristiques Techniques

- **Taille chunk max**: 64 bytes binaires (128 hex chars)
- **Timeout par défaut**: 5000 ms
- **CRC32 polynomial**: 0xEDB88320 (reflected form)
- **Baud rate UART**: 9600 (configurable)
- **Taille buffer**: 8192 bytes (configurable)
- **Format données**: Hexadécimal ASCII

## Dépendances

- Zephyr RTOS
- `CONFIG_UART_INTERRUPT_DRIVEN=y`
- `CONFIG_ARGOS_SMD=y`
- Heap suffisant (min 8KB recommandé)

## Améliorations Futures

- [ ] Support de compression firmware (LZMA, etc.)
- [ ] Chiffrement des données transférées
- [ ] Vérification de signature cryptographique
- [ ] Support de delta updates (patches)
- [ ] Reprise après interruption (resume)
- [ ] Support multi-bank flash (A/B partitioning)
- [ ] Interface de monitoring web/BLE
- [ ] Téléchargement firmware depuis cloud

## Références

- [Argos SMD Driver](drivers/argos-smd/argos_smd.c)
- [DFU API Header](include/argos-smd/argos_dfu.h)
- [Sample Application](samples/ota_test/)
- [Zephyr UART Driver](https://docs.zephyrproject.org/latest/hardware/peripherals/uart.html)

## Support

Pour des questions ou problèmes:
1. Consulter [samples/ota_test/README.md](samples/ota_test/README.md)
2. Activer logs niveau DEBUG: `CONFIG_ARGOS_DFU_LOG_LEVEL_DBG=y`
3. Vérifier configuration devicetree
4. Tester avec application d'exemple

## License

Copyright (c) 2025 Arribada Initiative
SPDX-License-Identifier: Apache-2.0
