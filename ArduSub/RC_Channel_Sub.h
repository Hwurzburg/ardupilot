#pragma once

#include <RC_Channel/RC_Channel.h>

class RC_Channel_Sub : public RC_Channel
{

public:

protected:
    __INITFUNC__ void init_aux_function(AUX_FUNC ch_option, AuxSwitchPos) override;
    bool do_aux_function(const AuxFuncTrigger &trigger) override;
    
private:

};

class RC_Channels_Sub : public RC_Channels
{
public:
    bool has_valid_input() const override;
    bool in_rc_failsafe() const override;
    RC_Channel_Sub obj_channels[NUM_RC_CHANNELS];
    RC_Channel_Sub *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

protected:

    // note that these callbacks are not presently used on Plane:
    int8_t flight_mode_channel_number() const override;

};
