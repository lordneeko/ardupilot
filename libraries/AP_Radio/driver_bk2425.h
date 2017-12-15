//----------------------------------------------------------------------------------
// low level driver for the Beken BK2425 radio on SPI
//----------------------------------------------------------------------------------

#pragma once

#include <AP_HAL/AP_HAL.h>

// Choose between supporting the Nordic nrf24l01+ and the Beken BK2425 radio chips
#define RADIO_NRF24 0
#define RADIO_BEKEN 1 // We are using the Beken BK2425 chip
#define SUPPORT_PA 0

/** SPI register commands for the BK2425 and nrf24L01+ chips */
typedef enum {
// General commands
	BK_REG_MASK        = 0x1F,  // The range of registers that can be read and written
	BK_READ_REG        = 0x00,  // Define read command to register (0..1F)
	BK_WRITE_REG       = 0x20,  // Define write command to register (0..1F)
#if RADIO_BEKEN
	BK_ACTIVATE_CMD	   = 0x50,
#endif
	BK_R_RX_PL_WID_CMD = 0x60,
	BK_RD_RX_PLOAD     = 0x61,  // Define RX payload register address
	BK_WR_TX_PLOAD     = 0xA0,  // Define TX payload register address
	BK_W_ACK_PAYLOAD_CMD = 0xA8, // (nrf: +pipe 0..7)
	BK_W_TX_PAYLOAD_NOACK_CMD = 0xB0,
	BK_FLUSH_TX        = 0xE1,  // Define flush TX register command
	BK_FLUSH_RX        = 0xE2,  // Define flush RX register command
	BK_REUSE_TX_PL     = 0xE3,  // Define reuse TX payload register command
	BK_NOP             = 0xFF,  // Define No Operation, might be used to read status register

// BK2425 bank 0 register addresses
	BK_CONFIG          = 0x00,  // 'Config' register address
	BK_EN_AA           = 0x01,  // 'Enable Auto Acknowledgment' register address
	BK_EN_RXADDR       = 0x02,  // 'Enabled RX addresses' register address
	BK_SETUP_AW        = 0x03,  // 'Setup address width' register address
	BK_SETUP_RETR      = 0x04,  // 'Setup Auto. Retrans' register address
	BK_RF_CH           = 0x05,  // 'RF channel' register address
	BK_RF_SETUP        = 0x06,  // 'RF setup' register address
	BK_STATUS          = 0x07,  // 'Status' register address
	BK_OBSERVE_TX      = 0x08,  // 'Observe TX' register address (lost packets, retransmitted packets on this frequency)
	BK_CD              = 0x09,  // 'Carrier Detect' register address
	BK_RX_ADDR_P0      = 0x0A,  // 'RX address pipe0' register address (5 bytes)
	BK_RX_ADDR_P1      = 0x0B,  // 'RX address pipe1' register address (5 bytes)
	BK_RX_ADDR_P2      = 0x0C,  // 'RX address pipe2' register address (1 byte)
	BK_RX_ADDR_P3      = 0x0D,  // 'RX address pipe3' register address (1 byte)
	BK_RX_ADDR_P4      = 0x0E,  // 'RX address pipe4' register address (1 byte)
	BK_RX_ADDR_P5      = 0x0F,  // 'RX address pipe5' register address (1 byte)
	BK_TX_ADDR         = 0x10,  // 'TX address' register address (5 bytes)
	BK_RX_PW_P0        = 0x11,  // 'RX payload width, pipe0' register address
	BK_RX_PW_P1        = 0x12,  // 'RX payload width, pipe1' register address
	BK_RX_PW_P2        = 0x13,  // 'RX payload width, pipe2' register address
	BK_RX_PW_P3        = 0x14,  // 'RX payload width, pipe3' register address
	BK_RX_PW_P4        = 0x15,  // 'RX payload width, pipe4' register address
	BK_RX_PW_P5        = 0x16,  // 'RX payload width, pipe5' register address
	BK_FIFO_STATUS     = 0x17,  // 'FIFO Status Register' register address
	BK_DYNPD           = 0x1c,  // 'Enable dynamic payload length' register address
	BK_FEATURE         = 0x1d,  // 'Feature' register address
#if RADIO_BEKEN
	BK_PAYLOAD_WIDTH   = 0x1f,  // 'payload length of 256 bytes modes register address

// BK2425 bank 1 register addresses
	BK2425_R1_4      = 0x04,
	BK2425_R1_5      = 0x05,
	BK2425_R1_WHOAMI = 0x08, // Register to read that contains the chip id
	BK2425_R1_12     = 0x0C, // PLL speed 120 or 130us
	BK2425_R1_13     = 0x0D,
	BK2425_R1_14     = 0x0E,
#endif
} BK_SPI_CMD;

//----------------------------------------------------------------------------------
// Chip Status Byte
//----------------------------------------------------------------------------------

enum {
	BK_CHIP_ID_BK2425 = 0x63, // The expected value of reading BK2425_R1_WHOAMI
};

// Meanings of the BK_STATUS register
enum {
#if RADIO_BEKEN
	BK_STATUS_RBANK = 0x80, // Register bank 1 is in use
#endif
	BK_STATUS_RX_DR = 0x40, // Data ready
	BK_STATUS_TX_DS = 0x20, // Data sent
	BK_STATUS_MAX_RT = 0x10, // Max retries failed
	BK_STATUS_RX_MASK = 0x0E, // Mask for the receptions bit
	BK_STATUS_RX_EMPTY = 0x0E,
	BK_STATUS_RX_P_5 = 0x0A, // Data pipe 5 has some data ready
	BK_STATUS_RX_P_4 = 0x08, // Data pipe 4 has some data ready
	BK_STATUS_RX_P_3 = 0x06, // Data pipe 3 has some data ready
	BK_STATUS_RX_P_2 = 0x04, // Data pipe 2 has some data ready
	BK_STATUS_RX_P_1 = 0x02, // Data pipe 1 has some data ready
	BK_STATUS_RX_P_0 = 0x00, // Data pipe 0 has some data ready
	BK_STATUS_TX_FULL = 0x01 // Tx buffer full
};

// Meanings of the FIFO_STATUS register
enum {
	BK_FIFO_STATUS_TX_REUSE = 0x40,
	BK_FIFO_STATUS_TX_FULL  = 0x20,
	BK_FIFO_STATUS_TX_EMPTY = 0x10,
	BK_FIFO_STATUS_RX_FULL  = 0x02,
	BK_FIFO_STATUS_RX_EMPTY = 0x01
};

// Meanings of the BK_CONFIG register
enum {
	BK_CONFIG_MASK_RX_DR = 0x40,  // Mask interrupt caused by RX_DR
	BK_CONFIG_MASK_TX_DS = 0x20,  // Mask interrupt caused by TX_DS
	BK_CONFIG_MASK_MAX_RT = 0x10, // Mask interrupt caused by MAX_RT
	BK_CONFIG_EN_CRC = 0x08,      // Enable CRC. Forced high if one of the bits in the EN_AA is high
	BK_CONFIG_CRCO = 0x04,        // CRC encoding scheme (0=8 bits, 1=16 bits)
	BK_CONFIG_PWR_UP = 0x02,      // POWER UP
	BK_CONFIG_PRIM_RX = 0x01,     // Receive/transmit
};

enum {
	BK_FEATURE_EN_DPL = 0x04,     //
	BK_FEATURE_EN_ACK_PAY = 0x02, //
	BK_FEATURE_EN_DYN_ACK = 0x01, //
};

#define BK_MAX_PACKET_LEN 32 // max value is 32 bytes
#define BK_RCV_TIMEOUT 30

//----------------------------------------------------------------------------------
// Translate output power into a number
// must match up with the table RegPower[]
#if RADIO_BEKEN
#define OUTPUT_POWER_REG6_0 0 // -25dB
#define OUTPUT_POWER_REG6_1 0 // -18dB
#define OUTPUT_POWER_REG6_2 1 // -18dB
#define OUTPUT_POWER_REG6_3 1 // -12dB
#define OUTPUT_POWER_REG6_4 1 // -12dB
#define OUTPUT_POWER_REG6_5 2 //  -7dB
#define OUTPUT_POWER_REG6_6 3 //  -1dB
#define OUTPUT_POWER_REG6_7 3 //  +4dB
#else // Nrf24 chip
#define OUTPUT_POWER_REG6_0 0 // -18dB
#define OUTPUT_POWER_REG6_1 0 // -18dB
#define OUTPUT_POWER_REG6_2 1 // -12dB
#define OUTPUT_POWER_REG6_3 1 // -12dB
#define OUTPUT_POWER_REG6_4 2 //  -6dB
#define OUTPUT_POWER_REG6_5 2 //  -6dB
#define OUTPUT_POWER_REG6_6 3 //  +0dB
#define OUTPUT_POWER_REG6_7 3 //  +0dB
#endif

// Register 4 in bank 1 only applies to Beken chip
#define OUTPUT_POWER_REG4_0 0 // -25dB
#define OUTPUT_POWER_REG4_1 3 // -18dB
#define OUTPUT_POWER_REG4_2 0 // -18dB
#define OUTPUT_POWER_REG4_3 3 // -12dB
#define OUTPUT_POWER_REG4_4 2 // -12dB
#define OUTPUT_POWER_REG4_5 0 //  -7dB
#define OUTPUT_POWER_REG4_6 0 //  -1dB
#define OUTPUT_POWER_REG4_7 7 //  +4dB

// Generic support
#define TOKENPASTE(x, y) x ## y
#define TOKENPASTE2(x, y) TOKENPASTE(x, y)
// The default register values that are for the default power setting
#define DEFAULT_OUTPUT_REG6 TOKENPASTE2(OUTPUT_POWER_REG6_,DEFAULT_OUTPUT_POWER)
#define DEFAULT_OUTPUT_REG4 TOKENPASTE2(OUTPUT_POWER_REG4_,DEFAULT_OUTPUT_POWER)

//----------------------------------------------------------------------------------
// Support the IO registers
#define BEKEN_SELECT()       hal.gpio->write_high(BEKEN_CS_GPIO_PORT, BEKEN_CS_PIN)
#define BEKEN_DESELECT()    do { \
								hal.gpio->write_high(BEKEN_CS_GPIO_PORT, BEKEN_CS_PIN); \
								nop(); \
								nop(); \
							} while(0)

#define BEKEN_CE_HIGH()      hal.gpio->write_high(BEKEN_CE_GPIO_PORT, BEKEN_CE_PIN)
#define BEKEN_CE_LOW()       hal.gpio->write_low(BEKEN_CE_GPIO_PORT, BEKEN_CE_PIN)

#if SUPPORT_PA
#define BEKEN_PA_HIGH()     (RADIO_TX_PORT->ODR |= RADIO_TX_PIN)
#define BEKEN_PA_LOW()      (RADIO_TX_PORT->ODR &= ~RADIO_TX_PIN)
#else
#define BEKEN_PA_HIGH()     ((void)0)
#define BEKEN_PA_LOW()      ((void)0)
#endif

//----------------------------------------------------------------------------------
// BEKEN driver class
class Radio_Beken {
public:
    Radio_Beken(AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev);

    void ReadFifo(uint8_t *dpbuffer, uint8_t len);
    void WriteFifo(const uint8_t *dpbuffer, uint8_t len);

    void ReadRegisterMulti(uint8_t address, uint8_t *data, uint8_t length);
    void WriteRegisterMulti(uint8_t address, const uint8_t *data, uint8_t length);
    void WriteRegisterMultiBank1(uint8_t address, const uint8_t *data, uint8_t length);

	uint8_t ReadStatus(void);
    uint8_t ReadReg(uint8_t reg);
    uint8_t Strobe(uint8_t address);
	void SetRBank(uint8_t bank);
    void WriteReg(uint8_t address, uint8_t data);
    void SetPower(uint8_t power);
    bool Reset(void);
	void SwitchToRxMode(void);
	void SwitchToTxMode(void);
	void SwitchToIdleMode(void);
	void SwitchToSleepMode(void);
	
    bool lock_bus(void) {
        return dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER);
    }
    void unlock_bus(void) {
        dev->get_semaphore()->give();
    }

private:
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
};
