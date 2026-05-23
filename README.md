# ESP32-C6 Window/Door Contact Sensor (Zigbee)

XIAO ESP32-C6 als Zigbee End-Device mit 2 Micro-Tastern für 3-Zustand-Fensterüberwachung (offen / gekippt / geschlossen). Sendet Custom-Cluster Attribute Reports an Zigbee2MQTT.

## Hardware

- **Board:** Seeed XIAO ESP32-C6
- **Taster 1:** Micro-Taster, D2 (= GPIO2) gegen GND
- **Taster 2:** Micro-Taster, D1 (= GPIO1) gegen GND
- Interner Pullup aktiv, active-low (gedrückter Taster = `true`)
- **Stromversorgung:** 2× AAA (~3 V), Battery-Setup mit Deep Sleep

> Beide Taster-Pins müssen RTC-fähig sein (GPIO 0–7 auf C6) — sonst kein EXT1 Wake im Deep Sleep.

## 3-Zustand-Erkennung: offen / gekippt / geschlossen

> 📸 **Bilder folgen** (Einbau im Fensterprofil, Taster-Positionen, fertiger Sensor)


Mit zwei Micro-Tastern lässt sich der Fensterstatus eindeutig unterscheiden.
**Verdeckter Einbau ins Fensterprofil — die Taster sind nach dem Einbau
unsichtbar** (Taster im festen Rahmen eingelassen, Stößel werden vom Flügel
mechanisch gedrückt; ESP + Batterie in der Falz oder verdeckt im Rahmen):

Jeder Taster wird durch die Mechanik in genau EINER Flügelstellung gedrückt
(die beiden Stellungen schließen sich gegenseitig aus — beide gleichzeitig
gedrückt kann nicht vorkommen):

```
Taster 1 (D2/GPIO2) — gedrückt wenn Flügel GESCHLOSSEN
Taster 2 (D1/GPIO1) — gedrückt wenn Flügel GEKIPPT
beide los              — Flügel OFFEN (Drehgriff in Offen-Stellung)
```

**Truth Table** (`true` = Taster gedrückt):

| `taster_1` | `taster_2` | Fensterstatus |
|---|---|---|
| `true`  | `false` | **geschlossen** |
| `false` | `true`  | **gekippt** |
| `false` | `false` | **offen** |
| `true`  | `true`  | (unmöglich — kommt mechanisch nicht vor) |

Die Auswertung der drei Zustände passiert in der Smarthome-Logik (z.B. ioBroker
Script oder z2m → HA Template Sensor), nicht im ESP-Firmware.

## Zigbee

- End-Device, joint via NETWORK_STEERING zu vorhandenem Netz
- Manufacturer: `ESPRESSIF`, Model: `esp32c6`
- Endpoint 10, Basic Cluster + Custom Cluster `0xFC00`
  - Attribute 0x0000 (BOOL) = `taster_1` (D2)
  - Attribute 0x0001 (BOOL) = `taster_2` (D1)
- Reports an Coordinator (`0x0000`, EP 1) bei GPIO-Änderung
- `ed_timeout` 256 min, `keep_alive` 10000 (sleepy end device)

## Deep Sleep

- Nach jedem Event 5 s Idle → `esp_deep_sleep_start()`
- Wake: EXT1 auf D2/D1 (per-pin level, beide Edges) ODER 1 h Timer-Fallback
- Letzte Pin-States in `RTC_DATA_ATTR` — spurious wakes (Floating-Noise) erkennen → kein Report, 500 ms Mini-Grace, sofort wieder Sleep
- Erster Join (nach Factory-Reset) hat 60 s Idle damit z2m Interview komplett wird
- Sleep-Strom typisch ~7 µA, Wake+Report ~2 s @ ~40 mA

## Build & Flash

ESP-IDF v5.3.2 + esp-zigbee-sdk benötigt.

```bash
cd ~/esp/window-sensor
source ~/esp/esp-idf/export.sh
idf.py set-target esp32c6
idf.py build
idf.py -p /dev/cu.usbmodem1101 flash
```

Factory-Reset (NVS löschen) vor neuem Pairing:
```bash
idf.py -p /dev/cu.usbmodem1101 erase-flash flash
```

## Pfad-Anpassungen

`CMakeLists.txt` und `main/idf_component.yml` enthalten absolute Pfade zu `~/esp/esp-zigbee-sdk/`. Bei anderer SDK-Location anpassen.

## Zigbee2MQTT Converter

External Converter (z2m 2.x → `data/external_converters/esp32c6_converter.js`):

```js
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const e = exposes.presets;
const access = exposes.access;

const definition = {
    zigbeeModel: ['esp32c6'],
    model: 'esp32c6',
    vendor: 'ESPRESSIF',
    description: 'ESP32-C6 Dual Taster',
    exposes: [
        e.binary('taster_1', access.STATE, true, false)
            .withDescription('Taster 1 (GPIO2)'),
        e.binary('taster_2', access.STATE, true, false)
            .withDescription('Taster 2 (GPIO1)'),
    ],
    fromZigbee: [{
        cluster: '64512',  // raw cluster ID as string (z2m matches by string or number; older zigbee-herdsman maps 64512 to 'manuSpecificAssaDoorLock' so that name also works on those setups)
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg) => {
            const result = {};
            if (msg.data.hasOwnProperty(0)) result.taster_1 = !!msg.data[0];
            if (msg.data.hasOwnProperty(1)) result.taster_2 = !!msg.data[1];
            return result;
        },
    }],
    toZigbee: [],
    configure: async () => {},
};

module.exports = definition;
```

**Hinweis Cluster-Name:** Cluster `0xFC00` (64512) ist je nach zigbee-herdsman Version unterschiedlich gemappt:
- Ältere Versionen registrieren es als `manuSpecificAssaDoorLock` → Converter mit String-Name funktioniert
- Neuere Versionen kennen den Cluster nicht namentlich → Frame kommt als `'64512'` rein → Converter braucht `cluster: '64512'`

Bei `"No converter available for 'esp32c6' with cluster '...'"` im z2m-Log den Cluster-Wert aus der Meldung in den Converter übernehmen.
