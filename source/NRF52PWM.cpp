#include "NRF52PWM.h"
#include "nrf.h"
#include "cmsis.h"
#include "CodalDmesg.h"

using namespace codal;

// TODO Experimenting with larger buffer to observe behave with extra few pulses bug
// #define  NRF52PWM_EMPTY_BUFFERSIZE  8
#define  NRF52PWM_EMPTY_BUFFERSIZE 64
static uint16_t emptyBuffer[NRF52PWM_EMPTY_BUFFERSIZE];

void nrf52_pwm0_irq(void)
{
    // Simply pass on to the driver component handler.
    if (NRF52PWM::nrf52_pwm_driver[0])
        NRF52PWM::nrf52_pwm_driver[0]->irq();
}

void nrf52_pwm1_irq(void)
{
    // Simply pass on to the driver component handler.
    if (NRF52PWM::nrf52_pwm_driver[1])
        NRF52PWM::nrf52_pwm_driver[1]->irq();
}

void nrf52_pwm2_irq(void)
{
    // Simply pass on to the driver component handler.
    if (NRF52PWM::nrf52_pwm_driver[2])
        NRF52PWM::nrf52_pwm_driver[2]->irq();
}

// Handles on the instances of this class used the three PWM modules (if present)
NRF52PWM* NRF52PWM::nrf52_pwm_driver[NRF52PWM_PWM_PERIPHERALS] = { NULL };

NRF52PWM::NRF52PWM(NRF_PWM_Type *module, DataSource &source, float sampleRate, uint16_t id) : PWM(*module), upstream(source)
{
    // initialise state
    this->id = id;
    this->dataReady = 0;
    this->active = false;
    this->streaming = true;
    this->repeatOnEmpty = true;
    this->bufferPlaying = 0;
    this->stopStreamingAfterBuf = 0;
    this->irqtotalcount = 0;

    // Clear empty buffer
    for (int i=0; i<NRF52PWM_EMPTY_BUFFERSIZE; i++)
        emptyBuffer[i] = 0x8000;

    // Ensure PWM is currently disabled.
    disable();

    // Configure hardware for requested sample rate.
    setSampleRate(sampleRate);

    // Configure for a repeating, edge aligned PWM pattern.
    PWM.MODE = PWM_MODE_UPDOWN_Up;

    // By default, enable control of four independent channels
    setDecoderMode(PWM_DECODER_LOAD_Individual);

    // Configure PWM for
    PWM.SEQ[1].REFRESH = 0;
    PWM.SEQ[0].REFRESH = 0;
    PWM.SEQ[1].ENDDELAY = 0;
    PWM.SEQ[0].ENDDELAY = 0;

    // Default to streaming mode.
    setStreamingMode(true);

    // Route an interrupt to this object
    // This is heavily unwound, but non trivial to remove this duplication given all the constants...
    // TODO: build up some lookup table to deduplicate this.
    if (&PWM == NRF_PWM0)
    {
        nrf52_pwm_driver[0] = this;
        NVIC_SetVector( PWM0_IRQn, (uint32_t) nrf52_pwm0_irq );
        NVIC_ClearPendingIRQ(PWM0_IRQn);
        NVIC_EnableIRQ(PWM0_IRQn);
    }

    if (&PWM == NRF_PWM1)
    {
        nrf52_pwm_driver[1] = this;
        NVIC_SetVector( PWM1_IRQn, (uint32_t) nrf52_pwm1_irq );
        NVIC_ClearPendingIRQ(PWM1_IRQn);
        NVIC_EnableIRQ(PWM1_IRQn);
    }

    if (&PWM == NRF_PWM2)
    {
        nrf52_pwm_driver[2] = this;
        NVIC_SetVector( PWM2_IRQn, (uint32_t) nrf52_pwm2_irq );
        NVIC_ClearPendingIRQ(PWM2_IRQn);
        NVIC_EnableIRQ(PWM2_IRQn);
    }

    // Enable the PWM module
    enable();

    // Register with our upstream component
    upstream.connect(*this);
}

/**
  * Clear the statistics, typically used before starting to output PWM
  * This is flawed due to not zeroing historical data
  */
void NRF52PWM::zeroStats() {
    // copy values to make history
    for (int i = IRQSTATSCOUNT - 1; i >= 0; i--) {
       memcpy(&stats[i], &stats[i - 1], sizeof(stats[0]));
    }

    // zero out current values
    stats[0].irqcount = stats[0].irq0 = stats[0].irq1 = stats[0].irqboth = 0;
    for (int i = 0; i < IRQSTATLEN; i++) {
        stats[0].irq_times[i] = 123456U;
        stats[0].pwmstart_time[i] = 123456U;
        stats[0].irq_seqend[i] = -1;
    }
    stats[0].irqinactive = 0;
    stats[0].irqprestart = 0;
    stats[0].irqpoststop = 0;
    stats[0].pwmstarts = stats[0].pwmstops = 0;
    stats[0].preloadfails = 0;
    stats[0].dataReadyAtStop = 123456;
    stats[0].nodata = 0;
    stats[0].irqtotalcountatstart = stats[0].irqtotalcountatstop = 0;
    stats[0].zero_time = DWT->CYCCNT;
}


/**
  * Check the statistics for anomalies and if any are found
  * DMESG print whole structure.
  */
void NRF52PWM::anomalyCheckStats(int bufcnt) {
    struct Nrf52PwmStats *s = &stats[0];
    int issues = 0;

    if (s->irqcount != (uint32_t)bufcnt)
        issues++;
    if (s->irqtotalcountatstop != s->irqtotalcountatstart + s->irqcount)
        issues++;
#if IRQSTATSCOUNT > 1 
    // Look for interrupts occuring while PWM should be not running
    if (s->irqtotalcountatstart != 0 && stats[1].irqtotalcountatstop != s->irqtotalcountatstart)
        issues++;
#endif
    if (s->irq0 + s->irq1 > s->irqcount || s->irqboth > 0)
        issues++;
    if (s->irqinactive > 0 || s->irqprestart > 0 || s->irqpoststop != 1)
        issues++; 
    if (s->pwmstarts != 1 || s->pwmstops != 1)
        issues++;
    if (s->preloadfails > 0 || s->dataReadyAtStop > 0 || s->nodata != 1)
        issues++;
    if (s->pwmstarts > 0) {
        // The wrapping time values require care with maths and comparisons
        uint32_t z_to_ps = s->pwmstart_time[0] - s->zero_time;
        if ((int32_t)z_to_ps <= 0)
            issues++;
        if (s->irqcount > 0) {
            uint32_t ps_to_irq = s->irq_times[0] - s->pwmstart_time[0];
            if ((int32_t)ps_to_irq <= 0)
                issues++;
        }
    }

    int testtrigger = 0;
    // if (DWT->CYCCNT % 1000 < 10)   // TODO - REMOVE JUST FOR TESTING
    //    testtrigger++;

    // if (issues > 0) {
    //     system_timer_wait_us(2000);  // TODO - remove, temporary to make more visible on logic analyzer
    //     return;
    // }
     
    if (issues > 0 || testtrigger) {
        DMESG("NRF52PWM anomalyCheckStart(%d) issues=%d", bufcnt, issues);
        for (int si = IRQSTATSCOUNT - 1; si >= 0; si--) {
            struct Nrf52PwmStats *s = &stats[si];
            DMESG("stats[%d]", si);
            DMESG("irqcount=%d irq0=%d irq1=%d irqboth=%d",
                s->irqcount, s->irq0, s->irq1, s->irqboth);
            DMESG("irqinactive=%d irqprestart=%d irqpoststop=%d pwmstarts=%s pwmstops=%d",
                s->irqinactive, s->irqprestart, s->irqpoststop, s->pwmstarts, s->pwmstops);
            DMESG("preloadfails=%d dataReadyAtStop=%d nodata=%d",
                s->preloadfails, s->dataReadyAtStop, s->nodata);
            DMESG("TS %u zero", s->zero_time);
            for (int i = 0; i < IRQSTATLEN && s->pwmstart_time[i] != 123456U; i++) {
                DMESG("TS %u pwmstart", s->pwmstart_time[i]);
            }
            for (int i = 0; i < IRQSTATLEN && s->irq_times[i] != 123456U; i++) {
                DMESG("TS %u irq %d", s->irq_times[i], s->irq_seqend[i]);
            }
            DMESGF("");
        }
    }
}


/**
 * Determine the DAC playback sample rate to the given frequency.
 * @return the current sample rate.
 */
float NRF52PWM::getSampleRate()
{
    return sampleRate;
}

/**
 * Determine the maximum unsigned vlaue that can be loaded into the PWM data values, for the current
 * frequency configuration.
 */
int NRF52PWM::getSampleRange()
{
    return PWM.COUNTERTOP;
}

/**
 * Change the DAC playback sample rate to the given frequency.
 * @param frequency The new sample playback frequency.
 */
float NRF52PWM::setSampleRate(float frequency)
{
    return setPeriodUs(1000000 / frequency);
}

/**
 * Change the DAC playback sample rate to the given period.
 * @param period The new sample playback period, in microseconds.
 */
int NRF52PWM::setPeriodUs(float period)
{
    int prescaler = 0;
    int clock_frequency = 16000000;
    int period_ticks = (clock_frequency/1000000) * period;

    // Calculate necessary prescaler.
    while(period_ticks >> prescaler >= 32768)
        prescaler++;

    // If the sample rate requested is outside the range of what the hardware can achieve, then there's nothign we can do.
    if (prescaler > 7)
        return DEVICE_INVALID_PARAMETER;

    // Decrement prescaler, as hardware indexes from zero.
    PWM.PRESCALER = prescaler;
    PWM.COUNTERTOP = period_ticks >> prescaler;

    // Update our internal record to reflect an accurate (probably rounded) samplerate.
    period_ticks = period_ticks >> prescaler;
    period_ticks = period_ticks << prescaler;

    periodUs = (float)period_ticks / (clock_frequency/1000000);
    sampleRate = 1000000 / periodUs;

    return DEVICE_OK;
}

/**
 * Determine the current DAC playback period.
 * @return period The sample playback period, in microseconds.
 */
float NRF52PWM::getPeriodUs()
{
    return periodUs;
}


/** 
 * Defines the mode in which the PWM module will operate, in terms of how it interprets data provided from the DataSource:
 * Valid options are:
 * 
 * PWM_DECODER_LOAD_Common          1st half word (16-bit) used in all PWM channels 0..3 
 * PWM_DECODER_LOAD_Grouped         1st half word (16-bit) used in channel 0..1; 2nd word in channel 2..3
 * PWM_DECODER_LOAD_Individual      1st half word (16-bit) in ch.0; 2nd in ch.1; ...; 4th in ch.3 
 * PWM_DECODER_LOAD_WaveForm        1st half word (16-bit) in ch.0; 2nd in ch.1; ...; 4th in COUNTERTOP
 * 
 * (See nrf52 product specificaiton for more details)
 * 
 * @param mode The mode for this PWM module to use.
 * @return DEVICE_OK, or DEVICE_INVALID_PARAMETER.
 */
int NRF52PWM::setDecoderMode(uint32_t mode)
{
    PWM.DECODER = (mode << PWM_DECODER_LOAD_Pos ) | (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos );

    return DEVICE_OK;
}
 
/**
 * Defines if the PWM module should maintain playout ordering of buffers, or always play the most recent buffer provided.
 * 
 * @ param streamingMode If true, buffers will be streamed in order they are received. If false, the most recent buffer supplied always takes prescedence.
 * @ param repeatOnEmpty If set to true, the last buffer received will be repeated when no data is available. If false, the PWM channel will be suspended.
 */
void NRF52PWM::setStreamingMode(bool streamingMode, bool repeatOnEmpty)
{
    this->streaming = streamingMode;
    this->repeatOnEmpty = repeatOnEmpty;
    setPwmLoopInten(streamingMode);
}

/**
 * Sets PWM hardware registers for chained buffers or simple one shot playback
 * 
 * @ param streamingMode If true, buffers will be streamed in order they are received. If false, the most recent buffer supplied always takes prescedence.
 */
void NRF52PWM::setPwmLoopInten(bool streamingMode) {
    if (streamingMode)
    {
        // Clearing events here to prevent any immediate spurious interrupts
        PWM.EVENTS_SEQEND[0] = PWM.EVENTS_SEQEND[1] = 0;
        PWM.LOOP = 1;
        PWM.SHORTS = PWM_SHORTS_LOOPSDONE_SEQSTART0_Enabled << PWM_SHORTS_LOOPSDONE_SEQSTART0_Pos; 
        PWM.INTENSET = (PWM_INTEN_SEQEND0_Enabled << PWM_INTEN_SEQEND0_Pos ) | (PWM_INTEN_SEQEND1_Enabled << PWM_INTEN_SEQEND1_Pos);
    }
    else
    {
        PWM.LOOP = 0;
        PWM.SHORTS = 0; 
        PWM.INTENCLR = (PWM_INTEN_SEQEND0_Enabled << PWM_INTEN_SEQEND0_Pos ) | (PWM_INTEN_SEQEND1_Enabled << PWM_INTEN_SEQEND1_Pos);
    }
}


/**
 * Pull a buffer into the given double buffer slot, if one is available.
 * @param b The buffer to fill (either 0 or 1)
 * @return the number of buffers succesfully filled.
 */
int NRF52PWM::tryPull(uint8_t b)
{
    if (stopStreamingAfterBuf)
    {
        // SHORTS must be disabled before STOP as "PWM could be immediately
        // started again if the LOOPSDONE event occurred in the same peripheral
        // clock cycle as the STOP task was triggered"
        setPwmLoopInten(false);  // includes SHORTS off
        PWM.TASKS_STOP = 1;
        while(PWM.EVENTS_STOPPED == 0);
        stats[0].irqtotalcountatstop = irqtotalcount;

        active = false;
        bufferPlaying = 0;
        stopStreamingAfterBuf = 0;

        stats[0].pwmstops++;

        upstream.dataWanted(DATASTREAM_NOT_WANTED);  // all the data has been PWMed

        // If a Pull request has been made since we decided to stop, start to fill up the
        // hardware double buffer so that we don't stall.
        stats[0].dataReadyAtStop = dataReady;
        if (dataReady > 0)
        {
            dataReady--;
            pullRequest();
        }
        return 0;
    }

    if (dataReady > 0) { 
        upstream.pull(buffer[b]);
        PWM.SEQ[b].PTR = (uint32_t) buffer[b].getBytes();
        PWM.SEQ[b].CNT = buffer[b].length() / 2;

        dataReady--;

        return 1;
    }

    // If we're in active streaming mode, and have requested a buffer and failed to get one, we have an underflow.
    // Streaming mode is double buffered, so schedule ourself to stop after the next buffer is played, if we're so configured.
    if (streaming && active && !repeatOnEmpty)
    {
        // The PWM doesn't seem to respond to changes in the SHORTS register while it's active...
        // instead, we provide an empty buffer to prevent partial repetition of any previous buffer.
        stats[0].nodata++;
        PWM.SEQ[b].PTR = (uint32_t) emptyBuffer;
        PWM.SEQ[b].CNT = (uint32_t) NRF52PWM_EMPTY_BUFFERSIZE;
        stopStreamingAfterBuf = 1;
    }
    return 0;
}

/**
 * Callback provided when data is ready.
 */
int NRF52PWM::pullRequest()
{
    dataReady++;

    // If we're not running in streaming mode, simply pull the requested buffer and schedule for DMA.
    if (!streaming)
    {
        int result = tryPull(0);
        if (result || repeatOnEmpty)
            PWM.TASKS_SEQSTART[0] = 1;
    }

    // If we're in streaming mode, ensure that we've preloaded both double buffers before initiating playout.
    // note: care needed here, as our upstream data source MAY recursively call pullRequest() again in response to us
    // pulling the first buffer...
    if (streaming && !active)
    {
        active = true;
        stats[0].irqtotalcountatstart = irqtotalcount;

        tryPull(bufferPlaying);
        bufferPlaying = (bufferPlaying + 1) % 2;

        if (bufferPlaying !=0 && dataReady > 0)
        {
            tryPull(bufferPlaying);
            bufferPlaying = (bufferPlaying + 1) % 2;
        }

        // Check if we've preloaded both buffers
        if (bufferPlaying == 0) {
            setPwmLoopInten(streaming);
            PWM.TASKS_SEQSTART[0] = 1;
            stats[0].pwmstart_time[stats[0].pwmstarts] = DWT->CYCCNT;
            stats[0].pwmstarts++;
        } else {
            active = false;
            stats[0].preloadfails++;
        }
    }

    return DEVICE_OK;
}

/**
 * Base implementation of a DMA callback
 */
void NRF52PWM::irq()
{
    size_t statidx = stats[0].irqcount % IRQSTATLEN;
    stats[0].irq_times[statidx] = DWT->CYCCNT;
    stats[0].irqcount++;
    irqtotalcount++;  // this counter will eventually wrap
    if (!active) {
        stats[0].irqinactive++;
    }
    if (stopStreamingAfterBuf) {
        stats[0].irqpoststop++;
    }
    if (stats[0].pwmstarts == 0) {
        stats[0].irqprestart++;
    }

    // once the sequence has finished playing, load up the next buffer.
    bool end0 = PWM.EVENTS_SEQEND[0];
    if (end0)
    {
        bufferPlaying = 1;
        tryPull(0);  // TODO log CYCCNT based duration in stats
        stats[0].irq0++;

        PWM.EVENTS_SEQEND[0] = 0;
    }

    bool end1 = PWM.EVENTS_SEQEND[1];
    if (end1)
    {
        bufferPlaying = 0;
        tryPull(1);  // TODO log CYCCNT based duration in stats
        stats[0].irq1++;

        PWM.EVENTS_SEQEND[1] = 0;
    }

    if (end0 && end1) {
        stats[0].irqboth++;
    }
    stats[0].irq_seqend[statidx] = (end1 ? 0b10 : 0) + (end0 ? 0b01 : 0);
}

/**
 * Enable this component
 */
void NRF52PWM::enable()
{
    enabled = true;
    PWM.ENABLE = 1;
}

/**
 * Disable this component
 */
void NRF52PWM::disable()
{
    enabled = false;
    PWM.ENABLE = 0;
}

/**
 * Direct output of given PWM channel to the given pin
 */
int
NRF52PWM::connectPin(Pin &pin, int channel)
{
    if (channel >= NRF52PWM_PWM_CHANNELS)
        return DEVICE_INVALID_PARAMETER;

    // If the pin is already connected to the requested channel, we have nothing to do.
    // We optimise this in order to prevent any possible glitch to an already connected channel...
    if (PWM.PSEL.OUT[channel] == pin.name)
        return DEVICE_OK;

    pin.disconnect();
    pin.setDigitalValue(0);
    PWM.PSEL.OUT[channel] = pin.name;
    pin.connect(*this);

    pin.status |= IO_STATUS_ANALOG_OUT;
    return DEVICE_OK;
}

/**
* Method to release the given pin from a peripheral, if already bound.
* Device drivers should override this method to disconnect themselves from the give pin
* to allow it to be used by a different peripheral.
*
* @param pin the Pin to be released
*/
int
NRF52PWM::releasePin(Pin &pin)
{
    for (int channel = 0; channel < NRF52PWM_PWM_CHANNELS; channel++)
        if (PWM.PSEL.OUT[channel] == pin.name)
            PWM.PSEL.OUT[channel] = 0xFFFFFFFF;

    if (deleteOnRelease)
        delete this;

    return DEVICE_OK;
}

int NRF52PWM::disconnectPin(Pin &pin)
{
    return releasePin(pin);
}
