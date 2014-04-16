#
# Gives feedback on Breakers
#

uint8 STATE_NOPOWER = 0
uint8 STATE_ENABLED = 1
uint8 STATE_DISABLED = 2

# The state of the breaker, should be one of the values seen above.
uint8 state

# The current flowing through the breaker, in Amps.
float32 current

# The temperature of the breaker, in degrees C.
float32 temperature
