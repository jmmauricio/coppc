"""Unit test package for colinker."""

from colinker.monitor import Monitor
import time

mon = Monitor('tests\config_devices_mininet_local.json','tests\config_controller.json')
mon.start()

#print(mon.measurements_dict)
print(mon.it)
print(mon.Time[-mon.it+2])

time.sleep(2)

#print(mon.measurements_dict)
print(mon.it)
print(mon.Time[-mon.it:]-mon.Time[-mon.it])
print(mon.Measurements[-mon.it:,0])

#print(mon.Time-mon.Time[-mon.it])

mon.monitoring = False