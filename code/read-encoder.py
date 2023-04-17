# Importing modules and classes
import time
import numpy as np
import gpiozero

ppr = 110  
tstop = 200  
tsample = 0.002  
tdisp = 0.002  
encoder = gpiozero.RotaryEncoder(17, 18, max_steps=0)
anglecurr = 0
tprev = 0
tcurr = 0
tstart = time.perf_counter()
while tcurr <= tstop:
    time.sleep(tsample)
    tcurr = time.perf_counter() - tstart

    anglecurr = 360 / ppr * encoder.steps
    if (np.floor(tcurr/tdisp) - np.floor(tprev/tdisp)) == 1:
        print("Angle = {:0.0f} deg".format(anglecurr))
    tprev = tcurr

print('Done.')





