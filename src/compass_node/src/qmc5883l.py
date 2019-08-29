import py_qmc5883l
import time

# requires py-qmc5883l lib: https://github.com/RigacciOrg/py-qmc5883l

sensor = py_qmc5883l.QMC5883L()

sensor.calibration = [[1.030, 0.026, -227.799],
                     [0.0255, 1.021, 1016.442],
                     [0.0, 0.0, 1.0]]

print(int(round(sensor.get_bearing())))
