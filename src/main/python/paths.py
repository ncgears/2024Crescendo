from trajectory_generator import trajectory_generator
from drive.swerve_drive import swerve_drive
import trajectory_io
import math

def main():
    drive = swerve_drive(
        # Wheelbase x/y (length/width)
        0.66, 0.584,
        # Bumper length/width (meters)
        0.914, 0.838,
        # Mass (kg)/moi (kg/m^2)
        54.4, 5.6,
        # Max velocity (RPS)/force (Nm) at 60A, adjusted for gearing
        95, 2.2, #1.1
        # 50, 1,
        # Wheel radius (meters)
        0.071)

    generator = trajectory_generator(drive)

    # array points are [xMeters,yMeters,rotationInRadians]
    # X is width of field, so -X is left, +X is right (from driver view)
    # Y is length of field, so -Y is toward driver station, +Y is away from driver station
    # Rotation is in radians, so 2pi/180 = 6.28rad, or a full rotation
    # if we setup offset from rotational zero, we need to account for this in our auton

    # generator.generate(
    #     [[0,2,0],
    #     [-1.0,0,-0.15]],
    #     "Test"
    # )

    # generator.generate(
    #     [[0,0,0],
    #     [1.0,0,0]],
    #     "OneMeterForward"
    # )

    # generator.generate(
    #     [[0,0,0],
    #     [-1.0,0,0]],
    #     "OneMeterBackward"
    # )

    # generator.generate(
    #     [[0,0,0],
    #     [2.0,0,0]],
    #     "TwoMetersForward"
    # )

    # generator.generate(
    #     [[0,0,0],
    #     [3.0,0,0]],
    #     "ThreeMetersForward"
    # )

    # generator.generate(
    #     [[0,0,0],
    #     [-3.0,0,0]],
    #     "ThreeMetersBackward"
    # )

    # generator.generate(
    #     [[0,0,0],
    #     [3.6,0,0]],
    #     "ThreeSixMetersForward"
    # )

    # generator.generate(
    #     [[0,0,0],
    #     [3.2,0,0]],
    #     "ThreeTwoMetersForward"
    # )

    # generator.generate(
    #     [[0,0,0],
    #     [4.0,0,0]],
    #     "FourMetersForward"
    # )

    generator.generate(
        [[0,0,0],
        [1.0,0.5,-0.5],
        [4.25,0.75,-0.65]],
        "RedOverBump"
    )

    # generator.generate(
    #     [[0,0,0],
    #     [4.25,-1.5,0]],
    #     "BlueOverBump"
    # )

    # generator.generate(
    #     [[0,0,0],
    #     [-2.0,0,0]],
    #     "TwoMetersBackward"
    # )

    # generator.generate(
    #     [[-0.42,6.121,-1],
    #     [-0.2,0.5,-0.575]],
    #     "GoHome"
    # )

main()