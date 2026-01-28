# Adafruit Feather nRF52840 - Configuration pour Argos SMD

## 🔌 Pinout UART pour Argos SMD

### Configuration par Défaut (UART1)

| Signal | Pin Feather | nRF52840 Pin | Description |
|--------|-------------|--------------|-------------|
| **TX** | D1 | P0.08 | UART TX → Argos SMD RX |
| **RX** | D0 | P0.06 | UART RX ← Argos SMD TX |
| **GND** | GND | GND | Ground commun |
| **VCC** | 3.3V | - | Alimentation Argos SMD |

### Connexions Recommandées

```
Feather nRF52840          Argos SMD Module
┌─────────────┐           ┌──────────────┐
│             │           │              │
│  D1 (TX) ───┼───────────┼──> RX       │
│  D0 (RX) <──┼───────────┼─── TX       │
│  GND ────────┼───────────┼─── GND      │
│  3.3V ───────┼───────────┼─── VCC      │
│             │           │              │
└─────────────┘           └──────────────┘
```

### Configuration Hardware Flow Control (Optionnel)

Si votre module Argos SMD supporte RTS/CTS:

| Signal | Pin Feather | nRF52840 Pin | Description |
|--------|-------------|--------------|-------------|
| **RTS** | D6 | P0.07 | Request To Send |
| **CTS** | A1 | P0.05 | Clear To Send |

Décommentez dans le fichier `.overlay`:
```dts
rts-pin = <7>;   /* P0.07 */
cts-pin = <5>;   /* P0.05 */
```

## 🎯 GPIO Wakeup (Mode Low Power)

Si vous utilisez le mode low power de l'Argos SMD, configurez un GPIO wakeup:

### Pins Disponibles

| Pin Feather | nRF52840 | Recommandé Pour |
|-------------|----------|-----------------|
| **D5** | P0.27 | Wakeup (recommandé) |
| **D9** | P0.26 | Wakeup alternatif |
| **A2** | P0.30 | Wakeup alternatif |
| **A3** | P0.28 | Wakeup alternatif |

### Configuration Exemple (D5 comme Wakeup)

Dans `adafruit_feather_nrf52840.overlay`:

```dts
argossmd {
    compatible = "arribada,argossmd";
    wakeup-gpios = <&gpio0 27 GPIO_ACTIVE_HIGH>;  /* D5 */
};
```

Connexion:
```
Feather D5 (P0.27) ──> Argos SMD Wakeup Pin
```

## 🔧 Configuration Devicetree

### Fichier: `boards/adafruit_feather_nrf52840.overlay`

```dts
&uart1 {
    status = "okay";
    current-speed = <9600>;
    tx-pin = <8>;    /* D1 */
    rx-pin = <6>;    /* D0 */

    argossmd {
        compatible = "arribada,argossmd";
        /* wakeup-gpios = <&gpio0 27 GPIO_ACTIVE_HIGH>; */
    };
};
```

## ⚙️ Build et Flash

### Build pour Feather

```bash
# Standard build
west build -b adafruit_feather_nrf52840 samples/ota_test

# Avec full OTA test
west build -b adafruit_feather_nrf52840 samples/ota_test -- \
  -DCONFIG_ARGOS_OTA_TEST_FULL_UPDATE=y

# Avec RTT logging
west build -b adafruit_feather_nrf52840 samples/ota_test -- \
  -DCONF_FILE=prj_rtt.conf
```

### Flash

```bash
# Via J-Link
west flash

# Via UF2 bootloader (si disponible)
# 1. Double-cliquez sur le bouton RESET
# 2. Copiez build/zephyr/zephyr.uf2 sur le lecteur FEATHERBOOT
```

## 🐛 Debug avec J-Link

La Feather nRF52840 supporte le debug via:
- **J-Link** via les pads SWD
- **Black Magic Probe**
- **OpenOCD**

### Connexions SWD

| Signal | Feather Pad | Description |
|--------|-------------|-------------|
| **SWDIO** | SWD pad | Data |
| **SWCLK** | SWC pad | Clock |
| **GND** | GND | Ground |
| **VTG** | 3.3V | Target voltage |

### Debug via VSCode

Appuyez sur `F5` ou utilisez:
```
Debug: OTA Test (Cortex-Debug)
```

## 📊 Monitoring Série

### UART Console (UART0)

Par défaut, UART0 est utilisé pour le shell/console:
- **TX**: P0.25
- **RX**: P0.24

Pour monitorer:
```bash
screen /dev/ttyACM0 115200
```

### RTT (Recommandé pour Debug)

Plus rapide et non-intrusif:
```bash
.vscode/rtt_viewer.sh client
```

## 🔋 Alimentation

### Options d'Alimentation

1. **USB** - 5V via USB-C
2. **Battery** - 3.7V LiPo sur connecteur JST
3. **3.3V** - Alimentation externe sur pin 3.3V

### Consommation Typique

- **Active (UART + Argos SMD)**: ~15-20 mA
- **Low Power Mode**: ~100 µA
- **Deep Sleep**: ~7 µA

## 🧪 Test de la Configuration

### Test 1: Vérifier UART

```c
// Dans main.c
#include <zephyr/drivers/uart.h>

const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart1));
if (!device_is_ready(uart)) {
    printk("ERROR: UART1 not ready!\n");
}
```

### Test 2: Vérifier Argos SMD

```bash
# Build et flash
west build -b adafruit_feather_nrf52840 samples/ota_test
west flash

# Monitorer RTT
.vscode/rtt_viewer.sh client

# Devrait afficher:
# "Argos SMD device ready"
# "--- Test: PING ---"
```

## 📝 Notes Importantes

### ⚠️ Niveau Logique

- Feather nRF52840: **3.3V logic**
- Argos SMD: **Vérifier le datasheet!**
- Si Argos SMD est 5V, utilisez un level shifter

### ⚠️ Pins Utilisés

Pins **NON disponibles** pour autres usages si UART1 actif:
- P0.06 (D0) - RX
- P0.08 (D1) - TX

### ⚠️ Courant Maximum

Pin 3.3V peut fournir jusqu'à **500 mA** sur Feather nRF52840.
Vérifiez la consommation du module Argos SMD!

## 🔗 Ressources

- [Feather nRF52840 Pinout](https://learn.adafruit.com/adafruit-feather-nrf52840-express)
- [nRF52840 Datasheet](https://infocenter.nordicsemi.com/pdf/nRF52840_PS_v1.1.pdf)
- [Zephyr Feather Board](https://docs.zephyrproject.org/latest/boards/adafruit/feather_nrf52840/doc/index.html)

## 🆘 Troubleshooting

### UART ne fonctionne pas

1. Vérifier les connexions TX/RX (croisées!)
2. Vérifier le baud rate (9600 par défaut)
3. Vérifier les pins dans `.overlay`
4. Tester avec un loopback (TX → RX)

### Flash échoue

1. Vérifier la connexion USB
2. Essayer le mode bootloader (double-clic RESET)
3. Vérifier les drivers J-Link

### Device not ready

1. Vérifier que `CONFIG_ARGOS_SMD=y`
2. Vérifier le devicetree overlay
3. Vérifier les logs: `CONFIG_ARGOS_SMD_LOG_LEVEL_DBG=y`
