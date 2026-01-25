const modern = require("zigbee-herdsman-converters/lib/modernExtend");
const {Zcl} = require('zigbee-herdsman');
const reporting = require("zigbee-herdsman-converters/lib/reporting");


const addCustomCluster = () =>
    modern.deviceAddCustomCluster('haCustomDehum', {
        ID: 0xFC10,
        manufacturerCode: 0x1307,
        attributes: {
            targetHum: {ID: 0x0000, type: Zcl.DataType.ENUM8, write: true, max: 0x0e},
            waterFull: {ID: 0x0001, type: Zcl.DataType.BOOLEAN},
            timerMins: {ID: 0x0002, type: Zcl.DataType.UINT16},
            timerSetHrs: {ID: 0x0003, type: Zcl.DataType.UINT8, write: true, max: 24},
            compressor: {ID: 0x0004, type: Zcl.DataType.BOOLEAN},
        },
        commands: {},
        commandsResponse: {},
    });

const definition = {
    fingerprint: [
        {
            modelID: 'KLOJA-DEHUM',
            manufacturerName: 'srett',
        }
    ],
    model: 'Dehumidifier',
    vendor: 'srett',
    description: 'Dehumidifier with Fan Control',
    extend: [
        addCustomCluster(),
        modern.onOff({powerOnBehavior: false}),
        modern.humidity(),
        modern.temperature(),
        modern.enumLookup({
            name: 'target_hum',
            lookup: {
                'AUTO': 0x00,
                'CONTINUOUS': 0x01,
                '30%': 0x02,
                '35%': 0x03,
                '40%': 0x04,
                '45%': 0x05,
                '50%': 0x06,
                '55%': 0x07,
                '60%': 0x08,
                '65%': 0x09,
                '70%': 0x0a,
                '75%': 0x0b,
                '80%': 0x0c,
                '85%': 0x0d,
                '90%': 0x0e,
            },
            cluster: 'haCustomDehum',
            attribute: 'targetHum',
            reporting: {min: 0, max: 0xFFFF, change: 1},
            label: 'Target humidity',
            description: 'Set the target humidity in %, automatic mode, or continuous operation',
        }),
        modern.enumLookup({
            name: 'fan_mode',
            // Can't use the fancontrol helpers for this, as we don't support on or off, and the HomeAssistant
            // integration will barf on it with an assetion failure around lib/extension/homeassistant.ts:938
            // assert(presets.length !== 0) - So instead of adding a nonsensical on or off option, just do this
            lookup: {
                'low': 0x01,
                'high': 0x03,
            },
            cluster: 'hvacFanCtrl',
            access: 'ALL',
            attribute: 'fanMode',
            reporting: {min: 0, max: 0xFFFF, change: 1},
            precision: 0,
            label: 'Fan speed',
            description: 'Speed at which the internal fan is running when device is turned on',
        }),
        modern.numeric({
            name: 'timer_mins',
            cluster: 'haCustomDehum',
            access: 'STATE_GET',
            attribute: 'timerMins',
            reporting: {min: 0, max: 0xFFFF, change: 1},
            precision: 0,
            unit: 'min',
            label: 'Current timer',
            description: 'Countdown in minutes until device turns on or off, 0 if disabled',
        }),
        modern.numeric({
            name: 'timer_set_hours',
            cluster: 'haCustomDehum',
            access: 'STATE_SET',
            attribute: 'timerSetHrs',
            precision: 0,
            unit: 'hours',
            label: 'Set timer',
            description: 'Start a new timer when the device should power on/off, in hours. Set to 0 to disable any current timer.',
        }),
        modern.binary({
            name: 'water_full',
            cluster: 'haCustomDehum',
            access: 'STATE_GET',
            attribute: 'waterFull',
            valueOn: ['Full', 1],
            valueOff: ['OK', 0],
            reporting: {min: 0, max: 0xFFFF, change: 0},
            label: 'Water full',
            description: 'The water tank is full',
        }),
        modern.binary({
            name: 'compressor',
            cluster: 'haCustomDehum',
            access: 'STATE_GET',
            attribute: 'compressor',
            valueOn: ['Running', 1],
            valueOff: ['Idle', 0],
            reporting: {min: 0, max: 0xFFFF, change: 0},
            label: 'Compressor',
            description: 'The compressor is running',
        }),
    ],
    fromZigbee: [],
    toZigbee: [],
    exposes: [],
    configure: async (device, coordinatorEndpoint, definition) => {
        const endpoint = device.getEndpoint(1);
        await reporting.bind(endpoint, coordinatorEndpoint, ['hvacFanCtrl', 'haCustomDehum']);
        // This one was necessary to get the fan mode reporting working, at least before turning it into an enum.
        // when it was using fz.*, tz.* and exposes.* helpers. Reporting didn't apply on its own, but this did the trick
        await endpoint.configureReporting(
            "hvacFanCtrl",
            [{attribute: "fanMode", minimumReportInterval: 0, maximumReportInterval: 3600, reportableChange: 1}]
        );
        // TODO: Figure out why z2m won't add reporting bindings for any of those attributes automatically or via those calls
        // Adding them by hand via the UI works as expected...
        await endpoint.configureReporting(
            "haCustomDehum",
            [{attribute: "targetHum", minimumReportInterval: 0, maximumReportInterval: 3600, reportableChange: 1}],
            {manufacturerCode: 0x1307}
        );
        await endpoint.configureReporting(
            "haCustomDehum",
            [{attribute: "waterFull", minimumReportInterval: 0, maximumReportInterval: 3600, reportableChange: 0}],
            {manufacturerCode: 0x1307}
        );
        await endpoint.configureReporting(
            "haCustomDehum",
            [{attribute: "timerMins", minimumReportInterval: 0, maximumReportInterval: 3600, reportableChange: 1}],
            {manufacturerCode: 0x1307}
        );
        await endpoint.configureReporting(
            "haCustomDehum",
            [{attribute: "compressor", minimumReportInterval: 0, maximumReportInterval: 3600, reportableChange: 0}],
            {manufacturerCode: 0x1307}
        );
    },
};

module.exports = definition;