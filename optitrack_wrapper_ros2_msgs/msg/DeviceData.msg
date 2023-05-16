# device ID (from data description)
int32 id

# number of active channels (signals) for this device
int32 n_channels

# channel (signal) data (e.g. ai0, ai1, ai2)
AnalogChannelData[] channel_data

# [b0:trigger bit] [b1:reserved] [b2:is synthetic data?] [b3,b4: mocap frame offset]
int16 params
