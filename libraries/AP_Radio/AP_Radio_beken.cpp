/*
  driver for Beken_2425 radio
 */
#include <AP_HAL/AP_HAL.h>

#pragma GCC optimize("O0")

#ifdef HAL_RCINPUT_WITH_AP_RADIO

#include <AP_Math/AP_Math.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <board_config.h>
#endif
#include "AP_Radio_cc2500.h"
#include <utility>
#include <stdio.h>
#include <StorageManager/StorageManager.h>
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
static THD_WORKING_AREA(_irq_handler_wa, 512);
#define TIMEOUT_PRIORITY 250	//Right above timer thread
#define EVT_TIMEOUT EVENT_MASK(0)
#define EVT_IRQ EVENT_MASK(1)
#define EVT_BIND EVENT_MASK(2)
#endif

extern const AP_HAL::HAL& hal;

#define Debug(level, fmt, args...)   do { if ((level) <= get_debug_level()) { hal.console->printf(fmt, ##args); }} while (0)

#define LP_FIFO_SIZE  16      // Physical data FIFO lengths in Radio

// object instance for trampoline
AP_Radio_beken *AP_Radio_beken::radio_instance;
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
thread_t *AP_Radio_beken::_irq_handler_ctx;
virtual_timer_t AP_Radio_beken::timeout_vt;
uint32_t AP_Radio_beken::irq_time_us;
#endif

/*
  constructor
 */
AP_Radio_beken::AP_Radio_beken(AP_Radio &_radio) :
    AP_Radio_backend(_radio),
    beken(hal.spi->get_device("beken")) // trace this later
{
    // link to instance for irq_trampoline
    radio_instance = this;
}

/*
  initialise radio
 */
bool AP_Radio_beken::init(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    if(_irq_handler_ctx != nullptr) {
        AP_HAL::panic("AP_Radio_beken: double instantiation of irq_handler\n");
    }
    chVTObjectInit(&timeout_vt);
    _irq_handler_ctx = chThdCreateStatic(_irq_handler_wa,
                     sizeof(_irq_handler_wa),
                     TIMEOUT_PRIORITY,        /* Initial priority.    */
                     irq_handler_thd,  /* Thread function.     */
                     NULL);                     /* Thread parameter.    */
#endif
    sem = hal.util->new_semaphore();    
    
    return reset();
}

/*
  reset radio
 */
bool AP_Radio_beken::reset(void)
{
    if (!beken.lock_bus()) {
        return false;
    }

    radio_init();
    beken.unlock_bus();

    return true;
}

/*
  return statistics structure from radio
 */
const AP_Radio::stats &AP_Radio_beken::get_stats(void)
{
    return stats;
}

/*
  read one pwm channel from radio
 */
uint16_t AP_Radio_beken::read(uint8_t chan)
{
    if (chan >= BEKEN_MAX_CHANNELS) {
        return 0;
    }
    return pwm_channels[chan];
}

/*
  update status - called from main thread
 */
void AP_Radio_beken::update(void)
{
}
    

/*
  return number of active channels
 */
uint8_t AP_Radio_beken::num_channels(void)
{
    uint32_t now = AP_HAL::millis();
    uint8_t chan = get_rssi_chan();
    if (chan > 0) {
        pwm_channels[chan-1] = t_status.rssi;
        chan_count = MAX(chan_count, chan);
    }

    chan = get_pps_chan();
    if (chan > 0) {
        pwm_channels[chan-1] = t_status.pps;
        chan_count = MAX(chan_count, chan);
    }

#if 0
    ch = get_tx_rssi_chan();
    if (ch > 0) {
        dsm.pwm_channels[ch-1] = dsm.tx_rssi;
        dsm.num_channels = MAX(dsm.num_channels, ch);
    }

    ch = get_tx_pps_chan();
    if (ch > 0) {
        dsm.pwm_channels[ch-1] = dsm.tx_pps;
        dsm.num_channels = MAX(dsm.num_channels, ch);
    }
#endif
    
    if (now - last_pps_ms > 1000) {
        last_pps_ms = now;
        t_status.pps = stats.recv_packets - last_stats.recv_packets;
        last_stats = stats;
        if (lost != 0 || timeouts != 0) {
            Debug(3,"lost=%u timeouts=%u\n", lost, timeouts);
        }
        lost=0;
        timeouts=0;
    }
    return chan_count;
}

/*
  return time of last receive in microseconds
 */
uint32_t AP_Radio_beken::last_recv_us(void)
{
    return packet_timer;
}

/*
  send len bytes as a single packet
 */
bool AP_Radio_beken::send(const uint8_t *pkt, uint16_t len)
{
    // disabled for now
    return false;
}

/*
const AP_Radio_beken::config AP_Radio_beken::radio_config[] = {
    {CC2500_02_IOCFG0,   0x01}, // GD0 high on RXFIFO filled or end of packet
    {CC2500_17_MCSM1,    0x0C}, // stay in RX on packet receive, CCA always, TX -> IDLE
    {CC2500_18_MCSM0,    0x18}, // XOSC expire 64, cal on IDLE -> TX or RX
    {CC2500_06_PKTLEN,   0x1E}, // packet length 30
    {CC2500_07_PKTCTRL1, 0x04}, // enable RSSI+LQI, no addr check, no autoflush, PQT=0
    {CC2500_08_PKTCTRL0, 0x01}, // var length mode, no CRC, FIFO enable, no whitening
    {CC2500_3E_PATABLE,  0xFF}, // ?? what are we doing to PA table here?
    {CC2500_0B_FSCTRL1,  0x0A}, // IF=253.90625kHz assuming 26MHz crystal
    {CC2500_0C_FSCTRL0,  0x00}, // freqoffs = 0
    {CC2500_0D_FREQ2,    0x5C}, // freq control high
    {CC2500_0E_FREQ1,    0x76}, // freq control middle
    {CC2500_0F_FREQ0,    0x27}, // freq control low
    {CC2500_10_MDMCFG4,  0x7B}, // data rate control
    {CC2500_11_MDMCFG3,  0x61}, // data rate control
    {CC2500_12_MDMCFG2,  0x13}, // 30/32 sync word bits, no manchester, GFSK, DC filter enabled
    {CC2500_13_MDMCFG1,  0x23}, // chan spacing exponent 3, preamble 4 bytes, FEC disabled
    {CC2500_14_MDMCFG0,  0x7A}, // chan spacing 299.926757kHz for 26MHz crystal
    {CC2500_15_DEVIATN,  0x51}, // modem deviation 25.128906kHz for 26MHz crystal
    {CC2500_19_FOCCFG,   0x16}, // frequency offset compensation
    {CC2500_1A_BSCFG,    0x6C}, // bit sync config
    {CC2500_1B_AGCCTRL2, 0x03}, // target amplitude 33dB
    {CC2500_1C_AGCCTRL1, 0x40}, // AGC control 2
    {CC2500_1D_AGCCTRL0, 0x91}, // AGC control 0
    {CC2500_21_FREND1,   0x56}, // frontend config1
    {CC2500_22_FREND0,   0x10}, // frontend config0
    {CC2500_23_FSCAL3,   0xA9}, // frequency synth cal3
    {CC2500_24_FSCAL2,   0x0A}, // frequency synth cal2
    {CC2500_25_FSCAL1,   0x00}, // frequency synth cal1
    {CC2500_26_FSCAL0,   0x11}, // frequency synth cal0
    //{CC2500_29_FSTEST,   0x59}, disabled FSTEST write
    {CC2500_2C_TEST2,    0x88}, // test settings
    {CC2500_2D_TEST1,    0x31}, // test settings
    {CC2500_2E_TEST0,    0x0B}, // test settings
    {CC2500_03_FIFOTHR,  0x07}, // TX fifo threashold 33, RX fifo threshold 32
    {CC2500_09_ADDR,     0x00}, // device address 0 (broadcast)
};
*/


extern const uint16_t CRCTable[]; 

/*
  initialise the radio
 */
void AP_Radio_beken::radio_init(void)
{
    //if (beken.ReadReg(CC2500_30_PARTNUM | CC2500_READ_BURST) != 0x80 ||
    //    beken.ReadReg(CC2500_31_VERSION | CC2500_READ_BURST) != 0x03) {
    //    return;
    //}

    Debug(1, "beken: radio_init starting\n");

    //beken.Reset();
    //for (uint8_t i=0; i<ARRAY_SIZE(radio_config); i++) {
    //    cc2500.WriteReg(radio_config[i].reg, radio_config[i].value);
    //}
    //cc2500.Strobe(CC2500_SIDLE);	// Go to idle...

    //for (uint8_t c=0;c<0xFF;c++) {
    //    //calibrate all channels
    //    cc2500.Strobe(CC2500_SIDLE);
    //    cc2500.WriteReg(CC2500_0A_CHANNR, c);
    //    cc2500.Strobe(CC2500_SCAL);
    //    hal.scheduler->delay_microseconds(900);
    //    calData[c][0] = cc2500.ReadReg(CC2500_23_FSCAL3);
    //    calData[c][1] = cc2500.ReadReg(CC2500_24_FSCAL2);
    //    calData[c][2] = cc2500.ReadReg(CC2500_25_FSCAL1);
    //}

    hal.scheduler->delay_microseconds(10*1000);
    
    // setup handler for rising edge of IRQ pin
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    stm32_gpiosetevent(CYRF_IRQ_INPUT, true, false, false, irq_radio_trampoline);
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    hal.gpio->attach_interrupt(HAL_GPIO_RADIO_IRQ, trigger_irq_radio_event, HAL_GPIO_INTERRUPT_RISING);
#endif

    if (load_bind_info()) {
        Debug(3,"Loaded bind info\n");
        listLength = 47;
        initialiseData(0);
        protocolState = STATE_SEARCH;
        chanskip = 1;
        nextChannel(1);
    } else {
        initTuneRx();
        protocolState = STATE_BIND_TUNING;
    }

    chVTSet(&timeout_vt, MS2ST(10), trigger_timeout_event, nullptr);
}

void AP_Radio_beken::trigger_irq_radio_event()
{
    //we are called from ISR context
    chSysLockFromISR();
    irq_time_us = AP_HAL::micros();
    chEvtSignalI(_irq_handler_ctx, EVT_IRQ);
    chSysUnlockFromISR();
}

void AP_Radio_beken::trigger_timeout_event(void *arg)
{
    (void)arg;
    //we are called from ISR context
    chSysLockFromISR();
    chVTSetI(&timeout_vt, MS2ST(10), trigger_timeout_event, nullptr);
    chEvtSignalI(_irq_handler_ctx, EVT_TIMEOUT);
    chSysUnlockFromISR();
}

void AP_Radio_cc2500::start_recv_bind(void)
{
    protocolState = STATE_BIND_TUNING;
    chan_count = 0;
    packet_timer = AP_HAL::micros();
    chEvtSignal(_irq_handler_ctx, EVT_BIND);
    Debug(1,"Starting bind\n");
}

// handle a data96 mavlink packet for fw upload
void AP_Radio_beken::handle_data_packet(mavlink_channel_t chan, const mavlink_data96_t &m)
{
}

// main IRQ handler
void AP_Radio_beken::irq_handler(void)
{
    uint8_t ccLen;
    //bool matched = false;
    //do {
    //    ccLen = cc2500.ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST);
    //    hal.scheduler->delay_microseconds(20);
    //    uint8_t ccLen2 = cc2500.ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST);
    //    matched = (ccLen == ccLen2);
    //} while (!matched);

    //if (ccLen & 0x80) {
    //    Debug(3,"Fifo overflow %02x\n", ccLen);
    //    // RX FIFO overflow
    //    cc2500.Strobe(CC2500_SFRX);
    //    cc2500.Strobe(CC2500_SRX);
    //    return;
    //}

    uint8_t packet[ccLen];
    beken.ReadFifo(packet, ccLen);

    if (get_fcc_test() != 0) {
        // don't process interrupts in FCCTEST mode
        return;
    }
    
#if 0
    if ((packet[ccLen-1] & 0x80) == 0) {
        // bad radio CRC
        return;
    }
#endif
    
#if 0
    static uint8_t counter;
    if (counter++ == 50) {
        Debug(2, "BEKEN IRQ state=%u\n", unsigned(protocolState));
        Debug(3,"len=%u\n", ccLen);
        for (uint8_t i=0; i<ccLen; i++) {
            Debug(4, "%02x:%02x ", i, packet[i]);
            if ((i+1) % 16 == 0) {
                Debug(4, "\n");
            }
        }
        if (ccLen % 16 != 0) {
            Debug(4, "\n");
        }
        counter = 0;
    }
#endif

    switch (protocolState) {
    case STATE_BIND_TUNING:
        if (tuneRx(ccLen, packet)) {
            Debug(2,"got BIND_TUNING\n");
            initGetBind();
            initialiseData(1);
            protocolState = STATE_BIND_BINDING1;
        }
        break;

    case STATE_BIND_BINDING1:
        if (getBind1(ccLen, packet)) {
            Debug(2,"got BIND1\n");
            protocolState = STATE_BIND_BINDING2;            
        }
        break;

    case STATE_BIND_BINDING2:
        if (getBind2(ccLen, packet)) {
            Debug(2,"got BIND2\n");
            protocolState = STATE_BIND_COMPLETE;
        }
        break;

    case STATE_BIND_COMPLETE:
        protocolState = STATE_STARTING;
        save_bind_info();
        Debug(3,"listLength=%u\n", listLength);
        Debug(3,"Saved bind info\n");
        break;

    case STATE_STARTING:
        listLength = 47;
        initialiseData(0);
        protocolState = STATE_SEARCH;
        chanskip = 1;
        nextChannel(1);
        break;

    case STATE_SEARCH:
        protocolState = STATE_DATA;
        // fallthrough

    case STATE_DATA: {
        if (packet[0] != 0x1D || ccLen != 32) {
            break;
        }
        if (!check_crc(ccLen, packet)) {
            Debug(3, "bad CRC\n");
            break;
        }
        if (packet[1] != bindTxId[0] ||
            packet[2] != bindTxId[1]) {
            Debug(3, "p1=%02x p2=%02x p6=%02x\n", packet[1], packet[2], packet[6]);
            // not for us
            break;
        }
        //if (packet[7] == 0x00 ||
        //    packet[7] == 0x20 ||
        //    packet[7] == 0x10 ||
        //    packet[7] == 0x12 ||
        //    packet[7] == 0x14 ||
        //    packet[7] == 0x16 ||
        //    packet[7] == 0x18 ||
        //    packet[7] == 0x1A ||
        //    packet[7] == 0x1C ||
        //    packet[7] == 0x1E) {
        //    // channel packet or range check packet
        //    parse_frSkyX(packet);
        //
        //    // get RSSI value from status byte
        //    uint8_t rssi_raw = packet[ccLen-2];
        //    float rssi_dbm;
        //    if (rssi_raw >= 128) {
        //        rssi_dbm = (rssi_raw - 256.0)/2;
        //    } else {
        //        rssi_dbm = rssi_raw*0.5;
        //    }
        //    rssi_filtered = 0.95 * rssi_filtered + 0.05 * rssi_dbm;
        //    t_status.rssi = uint8_t(rssi_filtered);
        //    
        //    stats.recv_packets++;
        //    uint8_t hop_chan = packet[4] & 0x3F;
        //    uint8_t skip = (packet[4]>>6) | (packet[5]<<2);
        //    if (channr != hop_chan) {
        //        Debug(4, "channr=%u hop_chan=%u\n", channr, hop_chan);
        //    }
        //    channr = hop_chan;
        //    if (chanskip != skip) {
        //        Debug(4, "chanskip=%u skip=%u\n", chanskip, skip);
        //    }
        //    chanskip = skip;
        //    packet_timer = irq_time_us;
        //    chVTSet(&timeout_vt, MS2ST(10), trigger_timeout_event, nullptr);
        //    
        //    packet3 = packet[3];
        //
        //    cc2500.Strobe(CC2500_SIDLE);
        //    cc2500.SetPower(get_transmit_power());
        //    send_telemetry();
        //
        //    // we can safely sleep here as we have a dedicated thread for radio processing.
        //    cc2500.unlock_bus();
        //    hal.scheduler->delay_microseconds(2800);
        //    cc2500.lock_bus();
        //
        //    nextChannel(chanskip);
        //    
        //} else {
        //    Debug(3, "p7=%02x\n", packet[7]);
        //}
        break;
    }

    case STATE_FCCTEST:
        // nothing to do, all done in timeout code
        Debug(3,"IRQ in FCCTEST state\n");
        break;

    default:
        Debug(2,"state %u\n", (unsigned)protocolState);
        break;
    }
}

// handle timeout IRQ
void AP_Radio_beken::irq_timeout(void)
{
    if (get_fcc_test() != 0 && protocolState != STATE_FCCTEST) {
        protocolState = STATE_FCCTEST;
        Debug(1,"Starting FCCTEST %u\n", get_fcc_test());
        setChannel(labs(get_fcc_test()) * 10);
        send_telemetry();
    }
    
    switch (protocolState) {
    case STATE_BIND_TUNING: {
        if (bindOffset >= 126) {
            bindOffset = -126;
        }
        uint32_t now = AP_HAL::millis();    
        if (now - timeTunedMs > 50) {
            timeTunedMs = now;
            bindOffset += 5;
            Debug(6,"bindOffset=%d\n", int(bindOffset));
            //cc2500.Strobe(CC2500_SIDLE);
            //cc2500.WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset);
            //cc2500.Strobe(CC2500_SFRX);
            //cc2500.Strobe(CC2500_SRX);
        }
        break;
    }
        
    case STATE_DATA: {
        uint32_t now = AP_HAL::micros();
        
        //if (now - packet_timer > 50*sync_time_us) {
        //    Debug(3,"searching %u\n", now - packet_timer);
        //    cc2500.Strobe(CC2500_SIDLE);
        //    cc2500.Strobe(CC2500_SFRX);
        //    nextChannel(1);
        //    cc2500.Strobe(CC2500_SRX);
        //    timeouts++;
        //    protocolState = STATE_SEARCH;
        //}
        break;
    }

    case STATE_SEARCH:
        // shift by one channel at a time when searching
        nextChannel(1);
        break;
            
    case STATE_FCCTEST: {
        if (get_fcc_test() == 0) {
            protocolState = STATE_DATA;
            Debug(1,"Ending FCCTEST\n");
        }
        setChannel(labs(get_fcc_test()) * 10);
        send_telemetry();
        chVTSet(&timeout_vt, MS2ST(10), trigger_timeout_event, nullptr);
        break;
    }

    default:
        break;
    }
}

void AP_Radio_beken::irq_handler_thd(void *arg)
{
    (void)arg;
    while(true) {
        eventmask_t evt = chEvtWaitAny(ALL_EVENTS);

        radio_instance->beken.lock_bus();
        
        switch(evt) {
        case EVT_IRQ:
            if (radio_instance->protocolState == STATE_FCCTEST) {
                hal.console->printf("IRQ FCC\n");
            }
            radio_instance->irq_handler();
            break;
        case EVT_TIMEOUT:
            //if (radio_instance->beken.ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST)) {
            //    irq_time_us = AP_HAL::micros();
            //    radio_instance->irq_handler();
            //} else {
            //    radio_instance->irq_timeout();
            //}
            break;
        case EVT_BIND:
            radio_instance->initTuneRx();
            break;
        default:
            break;
        }

        radio_instance->beken.unlock_bus();
    }
}

void AP_Radio_beken::initTuneRx(void)
{
    //cc2500.WriteReg(CC2500_19_FOCCFG, 0x14);
    //timeTunedMs = AP_HAL::millis();
    //bindOffset = -126;
    //cc2500.WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset);
    //cc2500.WriteReg(CC2500_07_PKTCTRL1, 0x0C);
    //cc2500.WriteReg(CC2500_18_MCSM0, 0x8);
    //
    //cc2500.Strobe(CC2500_SIDLE);
    //cc2500.WriteReg(CC2500_23_FSCAL3, calData[0][0]);
    //cc2500.WriteReg(CC2500_24_FSCAL2, calData[0][1]);
    //cc2500.WriteReg(CC2500_25_FSCAL1, calData[0][2]);
    //cc2500.WriteReg(CC2500_0A_CHANNR, 0);
    //cc2500.Strobe(CC2500_SFRX);
    //cc2500.Strobe(CC2500_SRX);
}

void AP_Radio_cc2500::initialiseData(uint8_t adr)
{
    //cc2500.WriteReg(CC2500_0C_FSCTRL0, bindOffset);
    //cc2500.WriteReg(CC2500_18_MCSM0, 0x8);
    //cc2500.WriteReg(CC2500_09_ADDR, adr ? 0x03 : bindTxId[0]);
    //cc2500.WriteReg(CC2500_07_PKTCTRL1, 0x0D); // address check, no broadcast, autoflush, status enable
    //cc2500.WriteReg(CC2500_19_FOCCFG, 0x16);
    hal.scheduler->delay_microseconds(10*1000);
}

void AP_Radio_cc2500::initGetBind(void)
{
    //cc2500.Strobe(CC2500_SIDLE);
    //cc2500.WriteReg(CC2500_23_FSCAL3, calData[0][0]);
    //cc2500.WriteReg(CC2500_24_FSCAL2, calData[0][1]);
    //cc2500.WriteReg(CC2500_25_FSCAL1, calData[0][2]);
    //cc2500.WriteReg(CC2500_0A_CHANNR, 0);
    //cc2500.Strobe(CC2500_SFRX);
    hal.scheduler->delay_microseconds(20); // waiting flush FIFO

    //cc2500.Strobe(CC2500_SRX);
    listLength = 0;
    bindIdx = 0x05;
}

/*
  check if we have received a packet with sufficiently good link quality
 */
bool AP_Radio_beken::tuneRx(uint8_t ccLen, uint8_t *packet)
{
    if (bindOffset >= 126) {
        bindOffset = -126;
    }
    if ((packet[ccLen - 1] & 0x80) && packet[2] == 0x01) {
        uint8_t Lqi = packet[ccLen - 1] & 0x7F;
        //bool crc_ok = (packet[ccLen - 1] & 0x80) != 0;
        if (Lqi < 50) {
            return true;
        }
    }
    return false;
}

/*
  get a block of hopping data
 */
bool AP_Radio_beken::getBind1(uint8_t ccLen, uint8_t *packet)
{
    // len|bind |tx
    // id|03|01|idx|h0|h1|h2|h3|h4|00|00|00|00|00|00|00|00|00|00|00|00|00|00|00|CHK1|CHK2|RSSI|LQI/CRC|
    // Start by getting bind packet 0 and the txid
    if ((packet[ccLen - 1] & 0x80) && packet[2] == 0x01 && packet[5] == 0x00) {
        bindTxId[0] = packet[3];
        bindTxId[1] = packet[4];
        for (uint8_t n = 0; n < 5; n++) {
            uint8_t c = packet[5] + n;
            if (c < sizeof(bindHopData)) {
                bindHopData[c] = packet[6 + n];
            }
        }
        return true;
    }
    return false;
}

/*
  get a 2nd stage of hopping data
 */
bool AP_Radio_beken::getBind2(uint8_t ccLen, uint8_t *packet)
{
    if (bindIdx > 120) {
        return true;
    }
    if ((packet[ccLen - 1] & 0x80) &&
        packet[2] == 0x01 &&
        packet[3] == bindTxId[0] &&
        packet[4] == bindTxId[1] &&
        packet[5] == bindIdx) {
        for (uint8_t n = 0; n < 5; n++) {
            if (packet[6 + n] == packet[ccLen - 3] || (packet[6 + n] == 0)) {
                if (bindIdx >= 0x2D) {
                    listLength = packet[5] + n;
                    return true;
                }
            }
            uint8_t c = packet[5] + n;
            if (c < sizeof(bindHopData)) {
                bindHopData[c] = packet[6 + n];
            }
        }
        bindIdx = bindIdx + 5;
        return false;
    }
    return false;
}

void AP_Radio_beken::setChannel(uint8_t channel)
{
    //cc2500.Strobe(CC2500_SIDLE);
    //cc2500.WriteReg(CC2500_23_FSCAL3, calData[channel][0]);
    //cc2500.WriteReg(CC2500_24_FSCAL2, calData[channel][1]);
    //cc2500.WriteReg(CC2500_25_FSCAL1, calData[channel][2]);
    //cc2500.WriteReg(CC2500_0A_CHANNR, channel);
    //cc2500.Strobe(CC2500_SRX);
}

void AP_Radio_cc2500::nextChannel(uint8_t skip)
{
    channr = (channr + skip) % listLength;
    setChannel(bindHopData[channr]);
}

void AP_Radio_beken::parse_frSkyX(const uint8_t *packet)
{
    uint16_t c[8];

    c[0] = (uint16_t)((packet[10] <<8)& 0xF00) | packet[9];
    c[1] = (uint16_t)((packet[11]<<4)&0xFF0) | (packet[10]>>4);
    c[2] = (uint16_t)((packet[13] <<8)& 0xF00) | packet[12];
    c[3] = (uint16_t)((packet[14]<<4)&0xFF0) | (packet[13]>>4);
    c[4] = (uint16_t)((packet[16] <<8)& 0xF00) | packet[15];
    c[5] = (uint16_t)((packet[17]<<4)&0xFF0) | (packet[16]>>4);
    c[6] = (uint16_t)((packet[19] <<8)& 0xF00) | packet[18];
    c[7] = (uint16_t)((packet[20]<<4)&0xFF0) | (packet[19]>>4);

    uint8_t j;
    for (uint8_t i=0;i<8;i++) {
        if(c[i] > 2047)  {
            j = 8;
            c[i] = c[i] - 2048;
        } else {
            j = 0;
        }
        if (c[i] == 0) {
            continue;
        }
        uint16_t word_temp = (((c[i]-64)<<1)/3+860);
        if ((word_temp > 800) && (word_temp < 2200)) {
            uint8_t chan = i+j;
            if (chan < CC2500_MAX_CHANNELS) {
                pwm_channels[chan] = word_temp;
                if (chan >= chan_count) {
                    chan_count = chan+1;
                }
            }
        }
    }
}

uint16_t AP_Radio_beken::calc_crc(uint8_t *data, uint8_t len)
{
    uint16_t crc = 0;
    for(uint8_t i=0; i < len; i++) {
        crc = (crc<<8) ^ (CRCTable[((uint8_t)(crc>>8) ^ *data++) & 0xFF]);
    }
    return crc;
}

bool AP_Radio_beken::check_crc(uint8_t ccLen, uint8_t *packet)
{
    uint16_t lcrc = calc_crc(&packet[3],(ccLen-7));
    return ((lcrc >>8)==packet[ccLen-4] && (lcrc&0x00FF)==packet[ccLen-3]);
}

/*
  save bind info
 */
void AP_Radio_beken::save_bind_info(void)
{
    // access to storage for bind information
    StorageAccess bind_storage(StorageManager::StorageBindInfo);
    struct bind_info info;
    
    info.magic = bind_magic;
    info.bindTxId[0] = bindTxId[0];
    info.bindTxId[1] = bindTxId[1];
    info.bindOffset = bindOffset;
    info.listLength = listLength;
    memcpy(info.bindHopData, bindHopData, sizeof(info.bindHopData));
    bind_storage.write_block(0, &info, sizeof(info));
}

/*
  load bind info
 */
bool AP_Radio_beken::load_bind_info(void)
{
    // access to storage for bind information
    StorageAccess bind_storage(StorageManager::StorageBindInfo);
    struct bind_info info;

    if (!bind_storage.read_block(&info, 0, sizeof(info)) || info.magic != bind_magic) {
        return false;
    }
    
    bindTxId[0] = info.bindTxId[0];
    bindTxId[1] = info.bindTxId[1];
    bindOffset = info.bindOffset;
    listLength = info.listLength;
    memcpy(bindHopData, info.bindHopData, sizeof(bindHopData));

    return true;
}

/*
  send a telemetry packet
 */
void AP_Radio_beken::send_telemetry(void)
{
    uint8_t frame[15];

    memset(frame, 0, sizeof(frame));
    
    frame[0] = sizeof(frame)-1;
    frame[1] = bindTxId[0];
    frame[2] = bindTxId[1];
    frame[3] = packet3;
    if (telem_send_rssi) {
        frame[4] = (t_status.rssi*2) | 0x80;
    } else {
        frame[4] = uint8_t(hal.analogin->board_voltage() * 10) & 0x7F;
    }
    telem_send_rssi = !telem_send_rssi;
    
    uint16_t lcrc = calc_crc(&frame[3], 10);
    frame[13] = lcrc>>8;
    frame[14] = lcrc;

    //cc2500.Strobe(CC2500_SIDLE);
    //cc2500.Strobe(CC2500_SFTX);
    if (get_fcc_test() >= 0) {
        // in negative FCC test modes we don't write to the FIFO, which gives
        // continuous transmission
        //cc2500.WriteFifo(frame, sizeof(frame));
    }
    //cc2500.Strobe(CC2500_STX);
}

#endif // HAL_RCINPUT_WITH_AP_RADIO

