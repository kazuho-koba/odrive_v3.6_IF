#!/usr/bin/env python3

import odrive

print("finding an odrive ...")
odrv0 = odrive.find_any()
print(str(odrv0.vbus_voltage))
