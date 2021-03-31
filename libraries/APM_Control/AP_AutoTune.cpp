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

// how much time we have to overshoot for to reduce gains
#define AUTOTUNE_OVERSHOOT_TIME 100

// how much time we have to undershoot for to increase gains
#define AUTOTUNE_UNDERSHOOT_TIME 200

// step size for increasing gains, percentage
#define AUTOTUNE_INCREASE_STEP 12

// step size for decreasing gains, percentage
#define AUTOTUNE_DECREASE_STEP 15

// min/max FF gains
#define AUTOTUNE_MAX_FF 2.0f
#define AUTOTUNE_MIN_FF 0.05f

// tau ranges
#define AUTOTUNE_MAX_TAU 0.7f
#define AUTOTUNE_MIN_TAU 0.2f

#define AUTOTUNE_MIN_IMAX 0.4
#define AUTOTUNE_MAX_IMAX 0.9

// constructor
AP_AutoTune::AP_AutoTune(ATGains &_gains, ATType _type,
                         const AP_Vehicle::FixedWing &parms,
                         AC_PID &_rpid) :
    current(_gains),
    rpid(_rpid),
    running(false),
    type(_type),
    aparm(parms),
    saturated_surfaces(false),
    ff_filter(1)
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
 */
static const struct {
    float tau;
    float rmax;
} tuning_table[] = {
    { 0.70f, 20 },   // level 1
    { 0.65f, 30 },   // level 2
    { 0.60f, 40 },   // level 3
    { 0.55f, 50 },   // level 4
    { 0.50f, 60 },   // level 5
    { 0.45f, 75 },   // level 6
    { 0.40f, 90 },   // level 7
    { 0.30f, 120 },   // level 8
    { 0.20f, 160 },   // level 9
    { 0.10f, 210 },   // level 10
    { 0.05f, 300 },   // (yes, it goes to 11)
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

    restore = last_save = get_gains(current);

    uint8_t level = aparm.autotune_level;
    if (level > ARRAY_SIZE(tuning_table)) {
        level = ARRAY_SIZE(tuning_table);
    }
    if (level < 1) {
        level = 1;
    }

    current.rmax.set(tuning_table[level-1].rmax);
    current.tau.set(tuning_table[level-1].tau);
    rpid.kIMAX().set(constrain_float(rpid.kIMAX(), AUTOTUNE_MIN_IMAX, AUTOTUNE_MAX_IMAX));

    next_save = current;

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
    }
}


/*
  one update cycle of the autotuner
 */
void AP_AutoTune::update(AP_Logger::PID_Info &pinfo, float scaler)
{
    if (!running) {
        return;
    }
    check_save();
    // see what state we are in
    ATState new_state = state;
    const float desired_rate = pinfo.target;
    const float actuator = pinfo.FF + pinfo.P + pinfo.I + pinfo.D;
    const float actual_rate = pinfo.actual;

    if (fabsf(actuator) >= 45) {
        // we have saturated the servo demand. We cannot get any
        // information that would allow us to increase the gain
        saturated_surfaces = true;
    }

    max_actuator = MAX(max_actuator, actuator);
    min_actuator = MIN(min_actuator, actuator);
    max_rate = MAX(max_rate, actual_rate);
    min_rate = MIN(min_rate, actual_rate);
    
    switch (state) {
    case ATState::IDLE:
        if (desired_rate > 0.8 * current.rmax) {
            new_state = ATState::DEMAND_POS;
        } else if (desired_rate < -0.8 * current.rmax) {
            new_state = ATState::DEMAND_NEG;
        }
        break;
    case ATState::DEMAND_POS:
        if (desired_rate < 0.2 * current.rmax) {
            new_state = ATState::IDLE;
        }
        break;
    case ATState::DEMAND_NEG:
        if (desired_rate > -0.2 * current.rmax) {
            new_state = ATState::IDLE;
        }
        break;
    }

    if (new_state == state) {
        return;
    }

    const uint32_t now = AP_HAL::millis();

    if (new_state != ATState::IDLE) {
        // starting an event
        min_actuator = max_actuator = min_rate = max_rate = 0;
        state_enter_ms = now;
        state = new_state;
        return;
    }

    if ((state == ATState::DEMAND_POS && max_rate < 0.01 * current.rmax) ||
        (state == ATState::DEMAND_NEG && min_rate > -0.01 * current.rmax)) {
        // we didn't get enough rate
        state = ATState::IDLE;
        return;
    }

    if (now - state_enter_ms < 200) {
        // not long enough sample
        state = ATState::IDLE;
        return;
    }

    // we've finished an event
    float ff;
    if (state == ATState::DEMAND_POS) {
        ff = max_actuator / (max_rate * scaler);
    } else {
        ff = min_actuator / (min_rate * scaler);
    }

    ff = ff_filter.apply(ff);

    const float old_ff = rpid.ff();

    // limit size of change
    ff = constrain_float(ff,
                         old_ff*(1-AUTOTUNE_DECREASE_STEP*0.01),
                         old_ff*(1+AUTOTUNE_INCREASE_STEP*0.01));

    Debug("FF=%.3f\n", ff);
    rpid.ff().set(ff);

    state = new_state;
    state_enter_ms = now;
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
    Debug("SAVE FF -> %.3f\n", rpid.ff().get());

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
    save_float_if_changed(current.tau, v.tau);
    save_int16_if_changed(current.rmax, v.rmax);
    save_float_if_changed(rpid.ff(), v.FF);
    save_float_if_changed(rpid.kP(), v.P);
    save_float_if_changed(rpid.kI(), v.I);
    save_float_if_changed(rpid.kD(), v.D);
    save_float_if_changed(rpid.kIMAX(), v.IMAX);
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

void AP_AutoTune::write_log(float servo, float demanded, float achieved)
{
    AP_Logger *logger = AP_Logger::get_singleton();
    if (!logger->logging_started()) {
        return;
    }

    struct log_ATRP pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATRP_MSG),
        time_us    : AP_HAL::micros64(),
        type       : static_cast<uint8_t>(type),
    	state      : (uint8_t)state,
        servo      : (int16_t)(servo*100),
        demanded   : demanded,
        achieved   : achieved,
        P          : rpid.kP().get()
    };
    logger->WriteBlock(&pkt, sizeof(pkt));
}
