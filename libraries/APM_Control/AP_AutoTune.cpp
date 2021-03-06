/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
   The strategy for roll/pitch autotune is to give the user a AUTOTUNE
   flight mode which behaves just like FBWA, but does automatic
   tuning.

   While the user is flying in AUTOTUNE the gains are saved every 10
   seconds, but the saved gains are not the current gains, instead it
   saves the gains from 10s ago. When the user exits AUTOTUNE the
   gains are restored from 10s ago.

   This allows the user to fly as much as they want in AUTOTUNE mode,
   and if they are ever unhappy they just exit the mode. If they stay
   in AUTOTUNE for more than 10s then their gains will have changed.

   Using this approach users don't need any special switches, they
   just need to be able to enter and exit AUTOTUNE mode
*/

#include "AP_AutoTune.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

// time in milliseconds between autotune saves
#define AUTOTUNE_SAVE_PERIOD 10000U

// step size for increasing gains, percentage
#define AUTOTUNE_INCREASE_FF_STEP 12
#define AUTOTUNE_INCREASE_PD_STEP 10

// step size for decreasing gains, percentage
#define AUTOTUNE_DECREASE_FF_STEP 15
#define AUTOTUNE_DECREASE_PD_STEP 20

// limits on IMAX
#define AUTOTUNE_MIN_IMAX 0.4
#define AUTOTUNE_MAX_IMAX 0.9

// ratio of I to P
#define AUTOTUNE_I_RATIO 0.75

// time constant of rate trim loop
#define TRIM_TCONST 1.0f

// overshoot threshold
#define AUTOTUNE_OVERSHOOT 1.1

// constructor
AP_AutoTune::AP_AutoTune(ATGains &_gains, ATType _type,
                         const AP_Vehicle::FixedWing &parms,
                         AC_PID &_rpid) :
    current(_gains),
    rpid(_rpid),
    type(_type),
    aparm(parms),
    ff_filter(2)
{}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>
# define Debug(fmt, args ...)  do {::printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

/*
  auto-tuning table. This table gives the starting values for key
  tuning parameters based on a user chosen AUTOTUNE_LEVEL parameter
  from 1 to 10. Level 1 is a very soft tune. Level 10 is a very
  aggressive tune.
  Level 0 means use the existing RMAX and TCONST parameters
 */
static const struct {
    float tau;
    float rmax;
} tuning_table[] = {
    { 1.00, 20 },   // level 1
    { 0.90, 30 },   // level 2
    { 0.80, 40 },   // level 3
    { 0.70, 50 },   // level 4
    { 0.60, 60 },   // level 5
    { 0.50, 75 },   // level 6
    { 0.30, 90 },   // level 7
    { 0.2, 120 },   // level 8
    { 0.15, 160 },   // level 9
    { 0.1, 210 },   // level 10
    { 0.1, 300 },   // (yes, it goes to 11)
};

/*
  start an autotune session
*/
void AP_AutoTune::start(void)
{
    running = true;
    state = ATState::IDLE;
    uint32_t now = AP_HAL::millis();

    last_save_ms = now;

    current = restore = last_save = get_gains(current);

    // do first update of rmax and tau now
    update_rmax();

    rpid.kIMAX().set(constrain_float(rpid.kIMAX(), AUTOTUNE_MIN_IMAX, AUTOTUNE_MAX_IMAX));

    next_save = current;

    // use 0.75Hz filters on the actuator, rate and target to reduce impact of noise
    actuator_filter.set_cutoff_frequency(AP::scheduler().get_loop_rate_hz(), 0.75);
    rate_filter.set_cutoff_frequency(AP::scheduler().get_loop_rate_hz(), 0.75);

    // target filter is a bit broader
    target_filter.set_cutoff_frequency(AP::scheduler().get_loop_rate_hz(), 4);

    ff_filter.reset();
    actuator_filter.reset();
    rate_filter.reset();

    if (!is_positive(rpid.slew_limit())) {
        // we must have a slew limit, default to 150 deg/s
        rpid.slew_limit().set_and_save(150);
    }

    if (current.FF < 0.01) {
        // don't allow for zero FF
        current.FF = 0.01;
        rpid.ff().set(current.FF);
    }

    Debug("START FF -> %.3f\n", rpid.ff().get());
}

/*
  called when we change state to see if we should change gains
 */
void AP_AutoTune::stop(void)
{
    if (running) {
        running = false;
        save_gains(restore);
        current = restore;
    }
}

/*
  one update cycle of the autotuner
 */
void AP_AutoTune::update(AP_Logger::PID_Info &pinfo, float scaler, float angle_err_deg)
{
    if (!running) {
        return;
    }
    check_save();
    // see what state we are in
    ATState new_state = state;
    const float desired_rate = target_filter.apply(pinfo.target);

    // filter actuator without I term so we can take ratios without
    // accounting for trim offsets. We first need to include the I and
    // clip to 45 degrees to get the right value of the real surface
    const float clipped_actuator = constrain_float(pinfo.FF + pinfo.P + pinfo.D + pinfo.I, -45, 45) - pinfo.I;
    const float actuator = actuator_filter.apply(clipped_actuator);
    const float actual_rate = rate_filter.apply(pinfo.actual);

    max_actuator = MAX(max_actuator, actuator);
    min_actuator = MIN(min_actuator, actuator);
    max_rate = MAX(max_rate, actual_rate);
    min_rate = MIN(min_rate, actual_rate);
    max_target = MAX(max_target, desired_rate);
    min_target = MIN(min_target, desired_rate);
    max_P = MAX(max_P, fabsf(pinfo.P));
    max_D = MAX(max_D, fabsf(pinfo.D));
    min_Dmod = MIN(min_Dmod, pinfo.Dmod);
    max_Dmod = MAX(max_Dmod, pinfo.Dmod);
    max_SRate = MAX(max_SRate, pinfo.slew_rate);

    float att_limit_deg;
    if (type == AUTOTUNE_ROLL) {
        att_limit_deg = aparm.roll_limit_cd * 0.01;
    } else {
        att_limit_deg = MIN(abs(aparm.pitch_limit_max_cd),abs(aparm.pitch_limit_min_cd))*0.01;
    }

    // thresholds for when we consider an event to start and end
    const float rate_threshold1 = 0.6 * MIN(att_limit_deg / current.tau.get(), current.rmax_pos);
    const float rate_threshold2 = 0.25 * rate_threshold1;
    bool in_att_demand = fabsf(angle_err_deg) >= 0.3 * att_limit_deg;

    switch (state) {
    case ATState::IDLE:
        if (desired_rate > rate_threshold1 && in_att_demand) {
            new_state = ATState::DEMAND_POS;
        } else if (desired_rate < -rate_threshold1 && in_att_demand) {
            new_state = ATState::DEMAND_NEG;
        }
        break;
    case ATState::DEMAND_POS:
        if (desired_rate < rate_threshold2) {
            new_state = ATState::IDLE;
        }
        break;
    case ATState::DEMAND_NEG:
        if (desired_rate > -rate_threshold2) {
            new_state = ATState::IDLE;
        }
        break;
    }

    const uint32_t now = AP_HAL::millis();

    if (now - last_log_ms >= 40) {
        // log at 25Hz
        struct log_ATRP pkt = {
            LOG_PACKET_HEADER_INIT(LOG_ATRP_MSG),
            time_us : AP_HAL::micros64(),
            type : uint8_t(type),
            state: uint8_t(new_state),
            actuator : actuator,
            desired_rate : desired_rate,
            actual_rate : actual_rate,
            FF_single: FF_single,
            FF: current.FF,
            P: current.P,
            I: current.I,
            D: current.D,
            action: uint8_t(action),
            rmax: float(current.rmax_pos.get()),
            tau: current.tau.get()
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
        last_log_ms = now;
    }

    if (new_state == state) {
        if (state == ATState::IDLE &&
            now - state_enter_ms > 500 &&
            max_Dmod < 0.9) {
            // we've been oscillating while idle, reduce P or D
            const float gain_mul = (100 - AUTOTUNE_DECREASE_PD_STEP)*0.01;
            if (max_P < max_D) {
                current.D *= gain_mul;
            } else {
                current.P *= gain_mul;
            }
            rpid.kP().set(current.P);
            rpid.kD().set(current.D);
            action = Action::IDLE_LOWER_PD;
            state_change(state);
        }
        return;
    }

    if (new_state != ATState::IDLE) {
        // starting an event
        min_actuator = max_actuator = min_rate = max_rate = 0;
        state_enter_ms = now;
        state = new_state;
        return;
    }

    if ((state == ATState::DEMAND_POS && max_rate < 0.01 * current.rmax_pos) ||
        (state == ATState::DEMAND_NEG && min_rate > -0.01 * current.rmax_neg)) {
        // we didn't get enough rate
        action = Action::LOW_RATE;
        state_change(ATState::IDLE);
        return;
    }

    if (now - state_enter_ms < 100) {
        // not long enough sample
        action = Action::SHORT;
        state_change(ATState::IDLE);
        return;
    }

    // we've finished an event. calculate the single-event FF value
    if (state == ATState::DEMAND_POS) {
        FF_single = max_actuator / (max_rate * scaler);
    } else {
        FF_single = min_actuator / (min_rate * scaler);
    }

    // apply median filter
    float FF = ff_filter.apply(FF_single);

    const float old_FF = rpid.ff();

    // limit size of change in FF
    FF = constrain_float(FF,
                         old_FF*(1-AUTOTUNE_DECREASE_FF_STEP*0.01),
                         old_FF*(1+AUTOTUNE_INCREASE_FF_STEP*0.01));

    // did the P or D components go over 30% of total actuator?
    const float abs_actuator = MAX(max_actuator, fabsf(min_actuator));
    const float PD_high = 0.3 * abs_actuator;
    bool PD_significant = (max_P > PD_high || max_D > PD_high);

    // see if we overshot
    const float dem_ratio  = (state == ATState::DEMAND_POS)?
        constrain_float(max_rate / max_target, 0.1, 2):
        constrain_float(min_rate / min_target, 0.1, 2);
    bool overshot = dem_ratio > AUTOTUNE_OVERSHOOT;

    // adjust P and D
    float D = rpid.kD();
    float P = rpid.kP();

    D = MAX(D, 0.0005);
    P = MAX(P, 0.01);

    // if the slew limiter kicked in or
    if (min_Dmod < 1.0 || (overshot && PD_significant)) {
        // we apply a gain reduction in proportion to the overshoot and dmod
        const float gain_mul = (100 - AUTOTUNE_DECREASE_PD_STEP)*0.01;
        const float dmod_mul = linear_interpolate(gain_mul, 1,
                                                  min_Dmod,
                                                  0.6, 1.0);
        const float overshoot_mul = linear_interpolate(1, gain_mul,
                                                       dem_ratio,
                                                       AUTOTUNE_OVERSHOOT, 1.3 * AUTOTUNE_OVERSHOOT);

        // we're overshooting or oscillating, decrease gains. We
        // assume the gain that needs to be reduced is the one that
        // peaked at a higher value
        if (max_P < max_D) {
            D *= dmod_mul * overshoot_mul;
        } else {
            P *= dmod_mul * overshoot_mul;
        }
        action = Action::LOWER_PD;
    } else {
        /* not oscillating or overshooting, increase the gains

           The increase is based on how far we are below the slew
           limit. At 80% of the limit we stop increasing gains, to
           give some margin. Below 25% of the limit we apply max
           increase
         */
        const float slew_limit = rpid.slew_limit();
        const float gain_mul = (100+AUTOTUNE_INCREASE_PD_STEP)*0.01;
        const float PD_mul = linear_interpolate(gain_mul, 1,
                                                max_SRate,
                                                0.2*slew_limit, 0.6*slew_limit);
        P *= PD_mul;
        D *= PD_mul;
        action = Action::RAISE_PD;
    }


    rpid.ff().set(FF);
    rpid.kP().set(P);
    rpid.kD().set(D);
    rpid.kI().set(MAX(P*AUTOTUNE_I_RATIO, (FF / TRIM_TCONST)));

    current.FF = FF;
    current.P = P;
    current.I = rpid.kI().get();
    current.D = D;

    Debug("FPID=(%.3f, %.3f, %.3f, %.3f)\n",
          rpid.ff().get(),
          rpid.kP().get(),
          rpid.kI().get(),
          rpid.kD().get());

    // move rmax and tau towards target
    update_rmax();

    state_change(new_state);
}

/*
  record a state change
 */
void AP_AutoTune::state_change(ATState new_state)
{
    min_Dmod = 1;
    max_Dmod = 0;
    max_SRate = 0;
    max_P = max_D = 0;
    state = new_state;
    state_enter_ms = AP_HAL::millis();
}

/*
  see if we should save new values
 */
void AP_AutoTune::check_save(void)
{
    if (AP_HAL::millis() - last_save_ms < AUTOTUNE_SAVE_PERIOD) {
        return;
    }

    // save the next_save values, which are the autotune value from
    // the last save period. This means the pilot has
    // AUTOTUNE_SAVE_PERIOD milliseconds to decide they don't like the
    // gains and switch out of autotune
    ATGains tmp = get_gains(current);

    save_gains(next_save);
    last_save = next_save;

    // restore our current gains
    set_gains(tmp);

    // if the pilot exits autotune they get these saved values
    restore = next_save;

    // the next values to save will be the ones we are flying now
    next_save = tmp;
    last_save_ms = AP_HAL::millis();
}

/*
  set a float and save a float if it has changed by more than
  0.1%. This reduces the number of insignificant EEPROM writes
 */
void AP_AutoTune::save_float_if_changed(AP_Float &v, float value)
{
    float old_value = v.get();
    v.set(value);
    if (value <= 0 || fabsf((value-old_value)/value) > 0.001f) {
        v.save();
    }
}

/*
  set a int16 and save if changed
 */
void AP_AutoTune::save_int16_if_changed(AP_Int16 &v, int16_t value)
{
    int16_t old_value = v.get();
    v.set(value);
    if (old_value != v.get()) {
        v.save();
    }
}


/*
  save a set of gains
 */
void AP_AutoTune::save_gains(const ATGains &v)
{
    ATGains tmp = current;
    current = last_save;
    save_float_if_changed(current.tau, v.tau);
    save_int16_if_changed(current.rmax_pos, v.rmax_pos);
    save_int16_if_changed(current.rmax_neg, v.rmax_neg);
    save_float_if_changed(rpid.ff(), v.FF);
    save_float_if_changed(rpid.kP(), v.P);
    save_float_if_changed(rpid.kI(), v.I);
    save_float_if_changed(rpid.kD(), v.D);
    save_float_if_changed(rpid.kIMAX(), v.IMAX);
    last_save = get_gains(current);
    current = tmp;
}

/*
  get gains with PID components
 */
AP_AutoTune::ATGains AP_AutoTune::get_gains(const ATGains &v)
{
    ATGains ret = v;
    ret.FF = rpid.ff();
    ret.P = rpid.kP();
    ret.I = rpid.kI();
    ret.D = rpid.kD();
    ret.IMAX = rpid.kIMAX();
    return ret;
}

/*
  set gains with PID components
 */
void AP_AutoTune::set_gains(const ATGains &v)
{
    current = v;
    rpid.ff().set(v.FF);
    rpid.kP().set(v.P);
    rpid.kI().set(v.I);
    rpid.kD().set(v.D);
    rpid.kIMAX().set(v.IMAX);
}

/*
  update RMAX and TAU parameters on each step. We move them gradually
  towards the target to allow for a user going straight to a level 10
  tune while starting with a poorly tuned plane
 */
void AP_AutoTune::update_rmax(void)
{
    uint8_t level = constrain_int32(aparm.autotune_level, 0, ARRAY_SIZE(tuning_table));

    int16_t target_rmax;
    float target_tau;

    if (level == 0) {
        // this level means to keep current values of RMAX and TCONST
        target_rmax = constrain_float(current.rmax_pos, 75, 720);
        target_tau = constrain_float(current.tau, 0.1, 2);
    } else {
        target_rmax = tuning_table[level-1].rmax;
        target_tau = tuning_table[level-1].tau;
    }

    if (level > 0 && is_positive(current.FF)) {
        const float invtau = ((1.0f / target_tau) + (current.I / current.FF));
        if (is_positive(invtau)) {
            target_tau = MAX(target_tau,1.0f / invtau);
        }
    }

    if (current.rmax_pos == 0) {
        // conservative initial value
        current.rmax_pos.set(75);
    }
    // move RMAX by 20 deg/s per step
    current.rmax_pos.set(constrain_int32(target_rmax,
                                         current.rmax_pos.get()-20,
                                         current.rmax_pos.get()+20));

    if (level != 0 || current.rmax_neg.get() == 0) {
        current.rmax_neg.set(current.rmax_pos.get());
    }

    // move tau by max 15% per loop
    current.tau.set(constrain_float(target_tau,
                                    current.tau*0.85,
                                    current.tau*1.15));
}
