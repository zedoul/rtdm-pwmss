#ifndef MF2044_ENC_LIB_H
#define MF2044_ENC_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	MF2044_ENC_PIN_NULL=0,
	MF2044_ENC_0=1 << 4,// P9.42 / P9.27
	MF2044_ENC_1=1 << 5,// P8.33 / P8.35
	MF2044_ENC_2=1 << 6,// P8.41 / P8.42
	MF2044_ENC_PIN_MAX,
} MF2044_ENC_PINS;

typedef enum {
	MF2044_ENC_MODE_NULL=0,
	MF2044_ENC_MODE_ABSOLUTE,// Absolute positioning mode
	MF2044_ENC_MODE_RELATIVE,// Relative positioning mode
	MF2044_ENC_MODE_MAX,
} MF2044_ENC_MODES;

// Open driver
int mf2044_enc_open(void);
// Close driver
int mf2044_enc_close(void);

// Get the mode of the eQEP hardware
MF2044_ENC_MODES mf2044_enc_mode_get(MF2044_ENC_PINS pin);
// Set the mode of the eQEP hardware
int mf2044_enc_mode_set(MF2044_ENC_PINS pin, MF2044_ENC_MODES mode);

// Get the period of the encoder
uint64_t mf2044_enc_period_get(MF2044_ENC_PINS pin);
// Set the period of the encoder
int mf2044_enc_period_set(MF2044_ENC_PINS pin, uint64_t period);

// Get the position of the encoder
int32_t mf2044_enc_position_get(MF2044_ENC_PINS pin);
// Set the position of the encoder
int mf2044_enc_position_set(MF2044_ENC_PINS pin, int32_t position);

//double mf2044_enc_velocity_get(MF2044_ENC_PINS pin);

#ifdef __cplusplus
}
#endif

#endif //MF2044_ENC_LIB_H
