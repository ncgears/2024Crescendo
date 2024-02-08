package frc.team1918.robot.utils;

import com.ctre.phoenix6.configs.AudioConfigs;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;
// import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

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
        Slot0Configs driveSlot0Configs = new Slot0Configs()
            .withKP(Constants.Swerve.kDriveKP)
            .withKI(Constants.Swerve.kDriveKI)
            .withKD(Constants.Swerve.kDriveKD);
        // TODO: figure out kV (again)
        // driveSlot0Configs.kV = Constants.SwerveK.kDriveKF;
        swerveDriveFXConfig.Slot0 = driveSlot0Configs;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.kDriveNeutralMode;
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.kRotationsPerWheelRotation;
        swerveDriveFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);

        //current limits
        CurrentLimitsConfigs driveCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.Swerve.kDriveCurrentLimitAmps)
            .withSupplyCurrentThreshold(Constants.Swerve.kDriveCurrentThresholdAmps)
            .withSupplyTimeThreshold(Constants.Swerve.kDriveCurrentThresholdSecs)
            .withSupplyCurrentLimitEnable(Constants.Swerve.kDriveCurrentLimitEnabled);
        swerveDriveFXConfig.CurrentLimits = driveCurrentLimitsConfigs;
        // TODO: check whether it is duty cycle or smtg else
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.kOpenLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.kClosedLoopRamp;

        //Shooter Configuration
        Slot0Configs shooterSlot0Configs = new Slot0Configs()
            .withKP(Constants.Shooter.kP)
            .withKI(Constants.Shooter.kI)
            .withKD(Constants.Shooter.kD)
            .withKV(Constants.Shooter.kV);
        shooterFXConfig.Slot0 = shooterSlot0Configs;
        shooterFXConfig.Voltage.PeakForwardVoltage = Constants.Shooter.kPeakFwdVoltage;
        shooterFXConfig.Voltage.PeakReverseVoltage = Constants.Shooter.kPeakRevVoltage;
        //Current Limits
        CurrentLimitsConfigs shooterCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.Shooter.kCurrentLimitAmps)
            .withSupplyCurrentThreshold(Constants.Shooter.kCurrentLimitThresholdAmps)
            .withSupplyTimeThreshold(Constants.Shooter.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(Constants.Shooter.kCurrentLimitEnable);
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

        //Aimer Configuration
        Slot0Configs aimerSlot0Configs = new Slot0Configs()
            .withKP(Constants.Aimer.kP)
            .withKI(Constants.Aimer.kI)
            .withKD(Constants.Aimer.kD)
            .withKV(Constants.Aimer.kV);
        aimerFXConfig.Slot0 = aimerSlot0Configs;
        aimerFXConfig.Voltage.PeakForwardVoltage = Constants.Aimer.kPeakFwdVoltage;
        aimerFXConfig.Voltage.PeakReverseVoltage = Constants.Aimer.kPeakRevVoltage;
        //Current Limits
        CurrentLimitsConfigs aimerCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.Aimer.kCurrentLimitAmps)
            .withSupplyCurrentThreshold(Constants.Aimer.kCurrentLimitThresholdAmps)
            .withSupplyTimeThreshold(Constants.Aimer.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(Constants.Aimer.kCurrentLimitEnable);
        aimerFXConfig.CurrentLimits = aimerCurrentLimitsConfigs;
        //Mechanical Limits
        //TODO: Move these to constants
        HardwareLimitSwitchConfigs aimerHardwareLimitsConfigs = new HardwareLimitSwitchConfigs()
            .withReverseLimitEnable(true)
            .withReverseLimitType(ReverseLimitTypeValue.NormallyClosed)
            .withReverseLimitAutosetPositionEnable(true)
            .withReverseLimitAutosetPositionValue(0.0)
            .withForwardLimitEnable(true)
            .withForwardLimitType(ForwardLimitTypeValue.NormallyClosed); //TODO: Add autoset position on forward limit to appropriate number also.
        aimerFXConfig.HardwareLimitSwitch = aimerHardwareLimitsConfigs;
        //Neutral and Direction
        aimerFXConfig.MotorOutput.NeutralMode = Constants.Aimer.kNeutralMode;
        aimerFXConfig.MotorOutput.Inverted = (Constants.Aimer.kIsInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        //Rotational rate modifiers
        aimerFXConfig.Feedback.SensorToMechanismRatio = Constants.Aimer.kGearRatio;
        //Audio
        aimerFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);

        //Example for other TalonFX based systems
        
        // FeedForward Gains
        // See https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/Slot0Configs.html#kG
        // for more information on feedforward gain for elevators, arms, etc.
        // Elevators use GravityType Elevator_Static where gravity is constant
        // Arms use GravityType Arm_Cosine where it depends on mechanism position
        // kG is the gravity gain -512..512
        // kA is the Acceleration gain
        // kV is the Velocity gain
        // kS is the Static gain

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