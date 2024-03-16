import numpy as np

def interpolate_servoH(CV_H):

    # Defined CV_H and ServoH values
    CV_H_values = [-8, 0, 8]
    Servo_H_values = [540, 500, 450]

    # Perform the interpolation
    return np.interp(CV_H, CV_H_values, Servo_H_values)

def interpolate_servoV(CV_V):
    # Defined CV_V and ServoV values
    CV_V_values = [16.5, 20.25, 24]
    Servo_V_values = [100, 115, 130]

    # Perform the interpolation
    return np.interp(CV_V, CV_V_values, Servo_V_values)

# Get user input
CV_H = float(input("Enter CV_H value: "))
CV_V = float(input("Enter CV_V value: "))

ServoH = interpolate_servoH(CV_H)
ServoV = interpolate_servoV(CV_V)

# Round ServoH and ServoV to the nearest 5
ServoH_rounded = round(ServoH / 5) * 5
ServoV_rounded = round(ServoV / 5) * 5

print(f"For CV_H = {CV_H}, the interpolated ServoH value is {ServoH_rounded}")
print(f"For CV_V = {CV_V}, the interpolated ServoV value is {ServoV_rounded}")

