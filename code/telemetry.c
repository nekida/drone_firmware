typedef struct telemetryConfig_s {
    float gpsNoFixLatitude;
    float gpsNoFixLongitude;
    uint8_t telemetry_switch;               // Use aux channel to change serial output & baudrate( MSP / Telemetry ). It disables automatic switching to Telemetry when armed.
    uint8_t telemetry_inverted;             // Flip the default inversion of the protocol - Same as serialrx_inverted in rx.c, but for telemetry.
    frskyGpsCoordFormat_e frsky_coordinate_format;
    frskyUnit_e frsky_unit;
    uint8_t frsky_vfas_precision;
    uint8_t frsky_pitch_roll;
    uint8_t report_cell_voltage;
    uint8_t hottAlarmSoundInterval;
    uint8_t halfDuplex;
    smartportFuelUnit_e smartportFuelUnit;
    uint8_t ibusTelemetryType;
    uint8_t ltmUpdateRate;

#ifdef USE_TELEMETRY_SIM
    int16_t simLowAltitude;
    char simGroundStationNumber[16];
    char simPin[8];
    uint16_t simTransmitInterval;
    uint8_t simTransmitFlags;

    uint16_t accEventThresholdHigh;
    uint16_t accEventThresholdLow;
    uint16_t accEventThresholdNegX;
#endif
    struct {
        uint8_t extended_status_rate;
        uint8_t rc_channels_rate;
        uint8_t position_rate;
        uint8_t extra1_rate;
        uint8_t extra2_rate;
        uint8_t extra3_rate;
        uint8_t version;
    } mavlink;
} telemetryConfig_t;

PG_DECLARE(telemetryConfig_t, telemetryConfig);

#define PG_DECLARE(_type, _name)                                        \
    extern _type _name ## _System;                                      \
    extern _type _name ## _Copy;                                        \
    static inline const _type* _name(void) { return &_name ## _System; }\
    static inline _type* _name ## Mutable(void) { return &_name ## _System; }\
    struct _dummy                                                       \
    /**/
	
extern telemetryConfig_t telemetryConfig_System;
extern telemetryConfig_t telemetryConfig_Copy;
static inline const telemetryConfig_t *telemetryConfig (void) { return &telemetryConfig_System; }
static inline telemetryConfig_t *telemetryConfigMutable (void) { return &telemetryConfig_System; }