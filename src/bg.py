#!/usr/bin/env python3
from brachiograph import BrachioGraph

# Uncomment the definition you want to use.

# This is an example BrachioGraph definition. If you build a plotter as
# described in the "Get started" section of the documentation, this definition
# is likely to work well. However, you should work out your own servo
# angle/pulse-width values as described in "Improve the plotter calibration".


# angles in degrees and corresponding pulse-widths for the two arm servos

servo_1_angle_pws1 = [
    [-162, 2470],
    [-144, 2250],
    [-126, 2050],
    [-108, 1860],
    [ -90, 1690],
    [ -72, 1530],
    [ -54, 1350],
    [ -36, 1190],
    [ -18, 1010],
    [   0,  840],
    [  18,  640],
]

servo_2_angle_pws2 = [
    [  0,  660],
    [ 18,  840],
    [ 36, 1030],
    [ 54, 1180],
    [ 72, 1340],
    [ 90, 1490],
    [108, 1640],
    [126, 1830],
    [144, 2000],
    [162, 2200],
    [180, 2410],
]

bg = BrachioGraph(
    # the lengths of the arms
    inner_arm=8,
    outer_arm=8,
    # the drawing area
    bounds=(-8, 4, 8, 13),
    # angles in degrees and corresponding pulse-widths for the two arm servos
    servo_1_angle_pws=servo_1_angle_pws1,
    servo_2_angle_pws=servo_2_angle_pws2,
    # pulse-widths for pen up/down
    pw_down=1200,
    pw_up=1850,
)


if __name__ == '__main__':

    bg.vertical_lines()
    bg.horizontal_lines()
    bg.grid_lines()
    bg.box()
    bg.park()
    
