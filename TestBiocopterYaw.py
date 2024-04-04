import math

class feedback_t:
    pass

PDterms = feedback_t()

# Hardcoded values for PDterms
PDterms.zEn = True
PDterms.yawEn = True
PDterms.kpz = 0.3
PDterms.kdz = 0.6
PDterms.kiz = 0.1
PDterms.z_int_low = 0.05
PDterms.z_int_high = 0.15
PDterms.kpyaw = 1.5
PDterms.kdyaw = 0.1
PDterms.kiyaw = 0.1
PDterms.kiyawrate = 0.1 

# Hardcoded values for control inputs
fx = 0  # Example value for fx constrain 0 to 2 
fz = 0  # Example value for fz
tx = 0  # Example value for tx
tz = 0 # Example value for tz

z_integral = 0
altitude = 2
dt = 1000
altitudeVelocity = 0
temperature = 0
pitch = 0
roll = 0
yaw = 0 # yaw and desired yaw is in -1 to 1 other in radians should we map? --> 
pitchrate = 0
rollrate = 0
yawrate = 0
battery = 0

fx = max(0, min(fx, 2*math.pi))
fz = max(0, min(fz, 2*math.pi))
tz = max(0, min(tz, 2*math.pi))

# Z feedback
if PDterms.zEn:
    z_integral += (fz - altitude) * dt / 1000000.0 * PDterms.kiz
    z_integral = max(PDterms.z_int_low, min(z_integral, PDterms.z_int_high))
    fz = (fz - altitude) * PDterms.kpz - altitudeVelocity * PDterms.kdz + z_integral
print("z_integral:", z_integral)
print("fz:", fz)

yaw_error = 0
yaw_integral = 0
yaw_control = 0
yawrate_desired = 0
yawrate_error = 0
yawrate_integral = 0

def map_value(value, old_min, old_max, new_min, new_max):
    # Map value from old range to new range
    old_range = old_max - old_min
    new_range = new_max - new_min
    scaled_value = (value - old_min) / old_range
    new_value = new_min + (scaled_value * new_range)
    return new_value

if PDterms.yawEn:
    # new_tz = max(0, min(tz, 2*math.pi))
    # yaw_error = new_tz - yaw # tz is -1,1      yaw is radians 
    # yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
    # yaw_error = max(-math.pi / 3, min(yaw_error, math.pi / 3))

    # yaw_integral += yaw_error * (dt / 1000000.0) * PDterms.kiyaw
    # yaw_integral = max(-math.pi / 4, min(yaw_integral, math.pi / 4))

    # yawrate_desired = yaw_error + yaw_integral

    # yawrate_error = yawrate_desired * PDterms.kpyaw - yawrate

    # yawrate_integral += yawrate_error * (dt / 1000000.0) * PDterms.kiyaw

    # new_tz = (
    #     yawrate_desired * PDterms.kpyaw +
    #     yawrate_error * PDterms.kdyaw -
    #     yawrate * PDterms.kiyawrate +
    #     yawrate_integral
    # )
    new_tz = max(0, min(tz, 2*math.pi))
    yaw_error = tz - yaw
    yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
    yaw_error = max(-math.pi / 3, min(yaw_error, math.pi / 3))

    yaw_integral += yaw_error * (dt / 1000000.0) * PDterms.kiz
    yaw_control = (tz - yaw)*PDterms.kpyaw - (yawrate)*PDterms.kdyaw + yaw_integral;

    # yaw_control = (tz - yaw) * PDterms['kpyaw'] - yawrate * PDterms['kdyaw'] + yaw_integral
    # print("yaw_error: ", yaw_error)
    # print("yaw_integral: ", yaw_integral)
    # print("yawrate_integral: ", yawrate_integral)
    # print("tz: ", tz)
print("yaw_error:", yaw_error)
print("yaw_integral:", yaw_integral)
# print("yaw_control:", yaw_control)
print('new_tz', new_tz)
#180 - angle 

# forceZMap = max(0, min(fz, 2))
x_force1 = new_tz + fx #-1 to 1
x_force2 = -new_tz + fx
# forceXMap1 = max(0, min(x_force1, 2))
# forceXMap2 = max(0, min(x_force2, 2))
# print("forceZMap:", forceZMap)
print("x_force1:", x_force1)
print("x_force2:", x_force2)

# print("forceXMap1:", forceXMap1)
# print("forceXMap2:", forceXMap2)


fMotor1 = math.sqrt(x_force1**2 + fz**2)
fMotor2 = math.sqrt(x_force2**2 + fz**2)
fMotor1 = max(0, min(fMotor1, 1))  # Constrain fMotor1 between 0 and 1
fMotor2 = max(0, min(fMotor2, 1))  # Constrain fMotor2 between 0 and 1
print("fMotor1:", fMotor1)
print("fMotor2:", fMotor2)


servoDeg1 = math.atan2(fz, x_force1) * (180 / math.pi)
servoDeg2 = 180 - math.atan2(fz, x_force2) * (180 / math.pi)
# //180-servoDeg2
servoDeg1 = max(0, min(servoDeg1, 180))  # Constrain servoDeg1 between 0 and 180
servoDeg2 = max(0, min(servoDeg2, 180))  # Constrain servoDeg2 between 0 and 180
print("servoDeg1:", servoDeg1)
print("servoDeg2:", servoDeg2)
# if (servoDeg1 == servoDeg2): # check 
#     servoDeg2 = (servoDeg2 + 180) - (servoDeg1 * 2)


m1 = fMotor1
m2 = fMotor2

t1 = servoDeg1
t2 = servoDeg2



print("m1:", m1)
print("m2:", m2)
print("t1:",      )
print("t2:", t2)



