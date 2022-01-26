#include "MAX30100_main.h"

#ifndef min
#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })
#endif

void MAX30100_init(){
    beat.threshold      = BEATDETECTOR_MIN_THRESHOLD;
    beat.beatPeriod     = 0;
    beat.lastMaxValue   = 0;
    beat.tsLastBeat     = 0;
}
void MAX30100_update()
{   
    uint16_t ir;
    uint16_t red;
    MAX30100_readFifoData();
    if(MAX30100_getRawValues(&ir, &red)){
            float irACValue = DC_Remover(ir);
            float filtered_IRvalue = FilterBuLp1(-irACValue);
            MAX30100_checkForBeat(filtered_IRvalue);
    }
    

}
int64_t MAX30100_millis(){
    int64_t time = esp_timer_get_time();
    return (int64_t) time/1000;
}
float MAX30100_getRate(){
    if (beat.beatPeriod != 0) {
        return 1 / beat.beatPeriod * 1000 * 60;
    } 
    else return 0;
}
bool MAX30100_checkForBeat(float sample)
{
    bool beatDetected = false;

    switch (beatState) {
        case BEATDETECTOR_STATE_INIT:
            if (MAX30100_millis() > BEATDETECTOR_INIT_HOLDOFF) {
                beatState = BEATDETECTOR_STATE_WAITING;
            }
            break;

        case BEATDETECTOR_STATE_WAITING:
            if (sample > beat.threshold) {
                beat.threshold = min(sample, BEATDETECTOR_MAX_THRESHOLD);
                beatState = BEATDETECTOR_STATE_FOLLOWING_SLOPE;
            }

            // Tracking lost, resetting
            if (MAX30100_millis() - beat.tsLastBeat > BEATDETECTOR_INVALID_READOUT_DELAY) {
                beat.beatPeriod = 0;
                beat.lastMaxValue = 0;
            }

            MAX30100_decreaseThreshold();
            break;

        case BEATDETECTOR_STATE_FOLLOWING_SLOPE:
            if (sample < beat.threshold) {
                beatState = BEATDETECTOR_STATE_MAYBE_DETECTED;
            } else {
                beat.threshold = min(sample, BEATDETECTOR_MAX_THRESHOLD);
            }
            break;

        case BEATDETECTOR_STATE_MAYBE_DETECTED:
            if (sample + BEATDETECTOR_STEP_RESILIENCY < beat.threshold) {
                // Found a beat
                beatDetected = true;
                beat.lastMaxValue = sample;
                beatState = BEATDETECTOR_STATE_MASKING;
                float delta = MAX30100_millis() - beat.tsLastBeat;
                if (delta) {
                    beat.beatPeriod = BEATDETECTOR_BPFILTER_ALPHA * delta +
                            (1 - BEATDETECTOR_BPFILTER_ALPHA) * beat.beatPeriod;
                }
                // printf("BEAT BEAT BEAT\t");
                // printf("HRV2:%f\n", MAX30100_getRate());
                beat.tsLastBeat = MAX30100_millis();
            } else {
                beatState = BEATDETECTOR_STATE_FOLLOWING_SLOPE;
            }
            break;

        case BEATDETECTOR_STATE_MASKING:
            if (MAX30100_millis() - beat.tsLastBeat > BEATDETECTOR_MASKING_HOLDOFF) {
                beatState = BEATDETECTOR_STATE_WAITING;
            }
            MAX30100_decreaseThreshold();
            break;
    }

    return beatDetected;
}

void MAX30100_decreaseThreshold()
{
    // When a valid beat rate readout is present, target the
    if ( beat.lastMaxValue > 0 &&  beat.beatPeriod > 0) {
         beat.threshold -= beat.lastMaxValue * (1 - BEATDETECTOR_THRESHOLD_FALLOFF_TARGET) /
                ( beat.beatPeriod / BEATDETECTOR_SAMPLES_PERIOD);
    } else {
        // Asymptotic decay
         beat.threshold *= BEATDETECTOR_THRESHOLD_DECAY_FACTOR;
    }

    if ( beat.threshold < BEATDETECTOR_MIN_THRESHOLD) {
         beat.threshold = BEATDETECTOR_MIN_THRESHOLD;
    }
}