package frc.team1918.robot.utils;

import com.ctre.phoenix6.configs.AudioConfigs;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;
// import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

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
    public final TalonFXConfiguration shooterFXConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration aimerFXConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration armFXConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration climberFXConfig = new TalonFXConfiguration();

    public CTREConfigs() {
        /* Swerve Drive Motor Configuration */
        Slot0Configs driveSlot0Configs = new Slot0Configs();
        driveSlot0Configs.kP = Constants.Swerve.kDriveKP;
        driveSlot0Configs.kI = Constants.Swerve.kDriveKI;
        driveSlot0Configs.kD = Constants.Swerve.kDriveKD;
        // TODO: figure out kV (again)
        // driveSlot0Configs.kV = Constants.SwerveK.kDriveKF;
        swerveDriveFXConfig.Slot0 = driveSlot0Configs;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.kDriveNeutralMode;
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.kRotationsPerWheelRotation;
        swerveDriveFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);

        //current limits
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

        //Shooter Configuration
        Slot0Configs shooterSlot0Configs = new Slot0Configs();
        shooterSlot0Configs.kP = Constants.Shooter.kP;
        shooterSlot0Configs.kI = Constants.Shooter.kI;
        shooterSlot0Configs.kD = Constants.Shooter.kD;
        shooterSlot0Configs.kV = Constants.Shooter.kF;
        shooterFXConfig.Slot0 = shooterSlot0Configs;
        shooterFXConfig.Voltage.PeakForwardVoltage = Constants.Shooter.kPeakFwdVoltage;
        shooterFXConfig.Voltage.PeakReverseVoltage = Constants.Shooter.kPeakRevVoltage;
        //Shooter Current Limits
        CurrentLimitsConfigs shooterCurrentLimitsConfigs = new CurrentLimitsConfigs();
        shooterCurrentLimitsConfigs.SupplyCurrentLimit = Constants.Shooter.kCurrentLimitAmps;
        shooterCurrentLimitsConfigs.SupplyCurrentThreshold = Constants.Shooter.kCurrentLimitThresholdAmps;
        shooterCurrentLimitsConfigs.SupplyTimeThreshold = Constants.Shooter.kCurrentLimitThresholdSecs;
        shooterCurrentLimitsConfigs.SupplyCurrentLimitEnable = Constants.Shooter.kCurrentLimitEnable;
        shooterFXConfig.CurrentLimits = shooterCurrentLimitsConfigs;
        //Ramping (spinup/spindown)
        shooterFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Shooter.kOpenLoopRamp;
        shooterFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Shooter.kClosedLoopRamp;
        //Neutral and Direction
        shooterFXConfig.MotorOutput.NeutralMode = Constants.Shooter.kNeutralMode;
        shooterFXConfig.MotorOutput.Inverted = (Constants.Shooter.isInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        //Rotational rate modifiers
        shooterFXConfig.Feedback.SensorToMechanismRatio = Constants.Shooter.kGearRatio;
        //Audio
        shooterFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);

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