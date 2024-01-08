package frc.team1918.robot.utils;

// import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.team1918.robot.Constants;

// import frc.robot.Constants.SwerveK;

// import static frc.robot.Constants.ElevatorK.*;

public final class CTREConfigs {
    private static final class Container {
        public static final CTREConfigs INSTANCE = new CTREConfigs();
    }

    public static CTREConfigs Get() {
        return Container.INSTANCE;
    }

    public final TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();

    private CTREConfigs() {
        /* Swerve Drive Motor Configuration */
        Slot0Configs driveSlot0Configs = new Slot0Configs();
        driveSlot0Configs.kP = Constants.Swerve.kDriveKP;
        driveSlot0Configs.kI = Constants.Swerve.kDriveKI;
        driveSlot0Configs.kD = Constants.Swerve.kDriveKD;
        // TODO: figure out kV (again)
        // driveSlot0Configs.kV = Constants.SwerveK.kDriveKF;
        swerveDriveFXConfig.Slot0 = driveSlot0Configs;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.kDriveNeutralMode;

        CurrentLimitsConfigs driveCurrentLimitsConfigs = new CurrentLimitsConfigs();
        driveCurrentLimitsConfigs.SupplyCurrentLimit = Constants.Swerve.kDriveCurrentLimitAmps;
        swerveDriveFXConfig.CurrentLimits = driveCurrentLimitsConfigs;

        // TODO: check whether it is duty cycle or smtg else
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.kOpenLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.kClosedLoopRamp;

        // /* Swerve CANcoder Configuration */
        // // TODO: check whether 0To1 works
        // swerveCancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        // swerveCancoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.kInvertCancoder
        //         ? SensorDirectionValue.Clockwise_Positive
        //         : SensorDirectionValue.CounterClockwise_Positive;

        // TODO: figure out what this is
        // swerveCancoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        //Example for other TalonFX based systems
        // /* Elevator Left and Right Motor Configuration */
        // Slot0Configs rightSlot0Configs = new Slot0Configs();
        // rightSlot0Configs.kP = kP;
        // rightSlot0Configs.kD = kD;
        // rightSlot0Configs.kV = kV;
        // rightConfig.Slot0 = rightSlot0Configs;

        // rightConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kForwardLimit;
        // rightConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kReverseLimit;
        // rightConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = kEnableForwardLimit;
        // rightConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = kEnableReverseLimit;

        // CurrentLimitsConfigs elevCurrentLimitsConfigs = new CurrentLimitsConfigs();
        // elevCurrentLimitsConfigs.SupplyCurrentLimit = kContinuousCurrentLimit;
        // rightConfig.CurrentLimits = elevCurrentLimitsConfigs;
        // leftConfig.CurrentLimits = elevCurrentLimitsConfigs;

        // rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    }
}