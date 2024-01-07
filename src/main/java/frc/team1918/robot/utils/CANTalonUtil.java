// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1918.robot.utils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/** Add your docs here. */
@Deprecated(since="2024")
public class CANTalonUtil {
    public enum Usage { kAll, kOpenLoop, kPrimaryPidOnly, kMinimal; }
    //see https://docs.ctre-phoenix.com/en/latest/ch18_CommonAPI.html

    public static void SetTalonCANUsage(WPI_TalonSRX motor, Usage usage, boolean isFollower) {
        //if follower, reduce status 1 and 2 to 500ms
        if(isFollower) {
            motor.setStatusFramePeriod(1,500); //10, output, faults, limits
            motor.setStatusFramePeriod(2,500); //20, sensor position (0), sensor velocity (0), brushed current, sticky faults
        }
        switch (usage) {
            case kOpenLoop:
                motor.setStatusFramePeriod(3,100); //100, quadrature info
                motor.setStatusFramePeriod(4,500); //100, analog input, supply voltage, controller temp
                motor.setStatusFramePeriod(8,100); //100, pulse width info
                motor.setStatusFramePeriod(10,500); //100, motion profiling/motion magic info
                motor.setStatusFramePeriod(12,500); //100, sensor position (1), sensor velocity (1)
                motor.setStatusFramePeriod(13,100); //100, pid0 info
                motor.setStatusFramePeriod(14,500); //100, pid1 info
                motor.setStatusFramePeriod(21,50); //100, integrated sensor position (talonfx), integrated sensor velocity (talonfx)
            case kPrimaryPidOnly:
                motor.setStatusFramePeriod(3,500); //100, quadrature info
                motor.setStatusFramePeriod(4,500); //100, analog input, supply voltage, controller temp
                motor.setStatusFramePeriod(8,500); //100, pulse width info
                motor.setStatusFramePeriod(10,500); //100, motion profiling/motion magic info
                motor.setStatusFramePeriod(12,500); //100, sensor position (1), sensor velocity (1)
                motor.setStatusFramePeriod(13,100); //100, pid0 info
                motor.setStatusFramePeriod(14,500); //100, pid1 info
                motor.setStatusFramePeriod(21,500); //100, integrated sensor position (talonfx), integrated sensor velocity (talonfx)
            case kMinimal:
                motor.setStatusFramePeriod(3,500); //100, quadrature info
                motor.setStatusFramePeriod(4,500); //100, analog input, supply voltage, controller temp
                motor.setStatusFramePeriod(8,500); //100, pulse width info
                motor.setStatusFramePeriod(10,500); //100, motion profiling/motion magic info
                motor.setStatusFramePeriod(12,500); //100, sensor position (1), sensor velocity (1)
                motor.setStatusFramePeriod(13,500); //100, pid0 info
                motor.setStatusFramePeriod(14,500); //100, pid1 info
                motor.setStatusFramePeriod(21,500); //100, integrated sensor position (talonfx), integrated sensor velocity (talonfx)
            case kAll:
            default:
        }
    };
    public static void SetTalonCANUsage(WPI_TalonFX motor, Usage usage, boolean isFollower) {
        //if follower, reduce status 1 and 2 to 500ms
        if(isFollower) {
            motor.setStatusFramePeriod(1,500); //10, output, faults, limits
            motor.setStatusFramePeriod(2,500); //20, sensor position (0), sensor velocity (0), brushed current, sticky faults
        }
        switch (usage) {
            case kOpenLoop:
                motor.setStatusFramePeriod(3,500); //100, quadrature info
                motor.setStatusFramePeriod(4,500); //100, analog input, supply voltage, controller temp
                motor.setStatusFramePeriod(8,500); //100, pulse width info
                motor.setStatusFramePeriod(10,500); //100, motion profiling/motion magic info
                motor.setStatusFramePeriod(12,500); //100, sensor position (1), sensor velocity (1)
                motor.setStatusFramePeriod(13,500); //100, pid0 info
                motor.setStatusFramePeriod(14,500); //100, pid1 info
                motor.setStatusFramePeriod(21,500); //100, integrated sensor position (talonfx), integrated sensor velocity (talonfx)
            case kPrimaryPidOnly:
                motor.setStatusFramePeriod(3,500); //100, quadrature info
                motor.setStatusFramePeriod(4,500); //100, analog input, supply voltage, controller temp
                motor.setStatusFramePeriod(8,500); //100, pulse width info
                motor.setStatusFramePeriod(10,500); //100, motion profiling/motion magic info
                motor.setStatusFramePeriod(12,500); //100, sensor position (1), sensor velocity (1)
                motor.setStatusFramePeriod(13,100); //100, pid0 info
                motor.setStatusFramePeriod(14,500); //100, pid1 info
                motor.setStatusFramePeriod(21,100); //100, integrated sensor position (talonfx), integrated sensor velocity (talonfx)
            case kMinimal:
                motor.setStatusFramePeriod(3,500); //100, quadrature info
                motor.setStatusFramePeriod(4,500); //100, analog input, supply voltage, controller temp
                motor.setStatusFramePeriod(8,500); //100, pulse width info
                motor.setStatusFramePeriod(10,500); //100, motion profiling/motion magic info
                motor.setStatusFramePeriod(12,500); //100, sensor position (1), sensor velocity (1)
                motor.setStatusFramePeriod(13,500); //100, pid0 info
                motor.setStatusFramePeriod(14,500); //100, pid1 info
                motor.setStatusFramePeriod(21,500); //100, integrated sensor position (talonfx), integrated sensor velocity (talonfx)
            case kAll:
            default:
        }
    };
}
