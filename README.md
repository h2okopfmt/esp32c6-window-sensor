# ESP32-C6 Window/Door Contact Sensor (Zigbee)

XIAO ESP32-C6 als Zigbee End-Device mit 2 Reed-Kontakten für Fenster/Tür-Überwachung. Sendet Custom-Cluster Attribute Reports an Zigbee2MQTT.

## Hardware

- **Board:** Seeed XIAO ESP32-C6
- **Reed 1:** D2 (= GPIO2) gegen GND
- **Reed 2:** D10 (= GPIO18) gegen GND
- Interner Pullup aktiv, active-low (geschlossener Reed = `true`)

## Zigbee

- End-Device, joint via NETWORK_STEERING zu vorhandenem Netz
- Manufacturer: `ESPRESSIF`, Model: `esp32c6`
- Endpoint 10, Basic Cluster + Custom Cluster `0xFC00`
  - Attribute 0x0000 (BOOL) = `taster_1` (D2)
  - Attribute 0x0001 (BOOL) = `taster_2` (D10)
- Reports an Coordinator (`0x0000`, EP 1) bei GPIO-Änderung

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
            .withDescription('Taster 2 (GPIO18)'),
    ],
    fromZigbee: [{
        cluster: 'manuSpecificAssaDoorLock',
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

**Hinweis:** Cluster `0xFC00` ist in zigbee-herdsman als `manuSpecificAssaDoorLock` registriert (Konflikt mit `manuSpecificPhilips`). Im Converter daher String-Name nutzen, keine Zahl.
