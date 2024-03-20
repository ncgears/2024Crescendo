package frc.team1918.robot.utils;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;
// import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

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

    //TalonFX
    public final TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration shooterFXConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration aimerFXConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration armFXConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration climberFXConfig = new TalonFXConfiguration();
    //CANcoder
    public final CANcoderConfiguration aimerCCConfig = new CANcoderConfiguration();
    public final CANcoderConfiguration armCCConfig = new CANcoderConfiguration();
    public final CANcoderConfiguration climberCCConfig = new CANcoderConfiguration();

    public CTREConfigs() {
        /* Swerve Drive Motor Configuration */
        Slot0Configs driveSlot0Configs = new Slot0Configs()
            .withKP(Constants.Swerve.kDriveKP)
            .withKI(Constants.Swerve.kDriveKI)
            .withKD(Constants.Swerve.kDriveKD);
        // figure out kV (again)
        // driveSlot0Configs.kV = Constants.SwerveK.kDriveKF;
        swerveDriveFXConfig.Slot0 = driveSlot0Configs;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.kDriveNeutralMode;
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.kRotationsPerWheelRotation;
        swerveDriveFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);

        //current limits
        CurrentLimitsConfigs driveCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Swerve.kDriveStatorCurrentLimitAmps)
            .withStatorCurrentLimitEnable(Constants.Swerve.kDriveStatorCurrentLimitEnable)
            .withSupplyCurrentLimit(Constants.Swerve.kDriveCurrentLimitAmps)
            .withSupplyCurrentThreshold(Constants.Swerve.kDriveCurrentThresholdAmps)
            .withSupplyTimeThreshold(Constants.Swerve.kDriveCurrentThresholdSecs)
            .withSupplyCurrentLimitEnable(Constants.Swerve.kDriveCurrentLimitEnabled);
        swerveDriveFXConfig.CurrentLimits = driveCurrentLimitsConfigs;
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.kOpenLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.kClosedLoopRamp;

        //Shooter Configuration
        Slot0Configs shooterSlot0Configs = new Slot0Configs()
            .withKP(Constants.Shooter.kP)
            .withKI(Constants.Shooter.kI)
            .withKD(Constants.Shooter.kD)
            .withKV(Constants.Shooter.kV);
        shooterFXConfig.Slot0 = shooterSlot0Configs;
        //Current Limits
        CurrentLimitsConfigs shooterCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.Shooter.kCurrentLimitAmps)
            .withSupplyCurrentThreshold(Constants.Shooter.kCurrentLimitThresholdAmps)
            .withSupplyTimeThreshold(Constants.Shooter.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(Constants.Shooter.kCurrentLimitEnable);
        shooterFXConfig.CurrentLimits = shooterCurrentLimitsConfigs;
        // shooterFXConfig.Voltage.PeakForwardVoltage = Constants.Shooter.kPeakFwdVoltage;
        // shooterFXConfig.Voltage.PeakReverseVoltage = Constants.Shooter.kPeakRevVoltage;
        //Ramping (spinup/spindown)
        // shooterFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Shooter.kOpenLoopRamp;
        // shooterFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Shooter.kClosedLoopRamp;
        //Motion Magic
        MotionMagicConfigs shooterMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.Shooter.kMotionMagicCruise)
            .withMotionMagicAcceleration(Constants.Shooter.kMotionMagicAccel)
            .withMotionMagicJerk(Constants.Shooter.kMotionMagicJerk);
        shooterFXConfig.MotionMagic = shooterMotionMagicConfigs;
        //Neutral and Direction
        shooterFXConfig.MotorOutput.NeutralMode = Constants.Shooter.kNeutralMode;
        shooterFXConfig.MotorOutput.Inverted = (Constants.Shooter.isInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        //Rotational rate modifiers
        shooterFXConfig.Feedback.SensorToMechanismRatio = Constants.Shooter.kGearRatio;
        //Audio
        shooterFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);

        //Aimer Configuration
        //CANcoder
        aimerCCConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        aimerCCConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        aimerCCConfig.MagnetSensor.MagnetOffset = Constants.Aimer.kMagnetOffset;
        
        Slot0Configs aimerSlot0Configs = new Slot0Configs()
            .withKP(Constants.Aimer.kP)
            .withKI(Constants.Aimer.kI)
            .withKD(Constants.Aimer.kD)
            .withKS(Constants.Aimer.kS)
            .withKV(Constants.Aimer.kV)
            .withKA(Constants.Aimer.kA);
        aimerFXConfig.Slot0 = aimerSlot0Configs;
        //Current Limits
        CurrentLimitsConfigs aimerCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.Aimer.kCurrentLimitAmps)
            .withSupplyCurrentThreshold(Constants.Aimer.kCurrentLimitThresholdAmps)
            .withSupplyTimeThreshold(Constants.Aimer.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(Constants.Aimer.kCurrentLimitEnable);
        aimerFXConfig.CurrentLimits = aimerCurrentLimitsConfigs;
        // aimerFXConfig.Voltage.PeakForwardVoltage = Constants.Aimer.kPeakFwdVoltage;
        // aimerFXConfig.Voltage.PeakReverseVoltage = Constants.Aimer.kPeakRevVoltage;
        //Ramping (spinup/spindown) //not needed or wanted for MM
        // aimerFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Aimer.kOpenLoopRamp;
        // aimerFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Aimer.kClosedLoopRamp;
        //Motion Magic
        MotionMagicConfigs aimerMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.Aimer.kMotionMagicCruise)
            .withMotionMagicAcceleration(Constants.Aimer.kMotionMagicAccel)
            .withMotionMagicJerk(Constants.Aimer.kMotionMagicJerk);
        aimerFXConfig.MotionMagic = aimerMotionMagicConfigs;
        //Mechanical Limits
        SoftwareLimitSwitchConfigs aimerSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitEnable(Constants.Aimer.kSoftReverseLimitEnable)
            .withReverseSoftLimitThreshold(Constants.Aimer.kSoftReverseLimit)
            .withForwardSoftLimitEnable(Constants.Aimer.kSoftForwardLimitEnable)
            .withForwardSoftLimitThreshold(Constants.Aimer.kSoftForwardLimit);
        aimerFXConfig.SoftwareLimitSwitch = aimerSoftwareLimitSwitchConfigs;
        // HardwareLimitSwitchConfigs aimerHardwareLimitsConfigs = new HardwareLimitSwitchConfigs()
        //     .withReverseLimitEnable(false)
        //     .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
        //     .withReverseLimitAutosetPositionEnable(true)
        //     .withReverseLimitAutosetPositionValue(0.0)
        //     .withForwardLimitEnable(false)
        //     .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen); //Add autoset position on forward limit to appropriate number also.
        // aimerFXConfig.HardwareLimitSwitch = aimerHardwareLimitsConfigs;
        //Encoder
        if(Constants.Aimer.kUseCANcoder) {
            aimerFXConfig.Feedback.FeedbackRemoteSensorID = Constants.Aimer.kCANcoderID;
            aimerFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
            aimerFXConfig.Feedback.RotorToSensorRatio = Constants.Aimer.kGearRatio;
            aimerFXConfig.Feedback.SensorToMechanismRatio = 1.0; //CANcoder is the same as mechanism
        } else {
            aimerFXConfig.Feedback.SensorToMechanismRatio = Constants.Aimer.kGearRatio;
        }
        //Neutral and Direction
        aimerFXConfig.MotorOutput.NeutralMode = Constants.Aimer.kNeutralMode;
        aimerFXConfig.MotorOutput.Inverted = (Constants.Aimer.kIsInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        //Audio
        aimerFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);

        //Arm
        //CANcoder
        armCCConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        armCCConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        armCCConfig.MagnetSensor.MagnetOffset = Constants.Arm.kMagnetOffset;

        Slot0Configs armSlot0Configs = new Slot0Configs()
            .withKP(Constants.Arm.kP)
            .withKI(Constants.Arm.kI)
            .withKD(Constants.Arm.kD)
            .withKS(Constants.Arm.kS)
            .withKV(Constants.Arm.kV)
            .withKA(Constants.Arm.kA);
        armFXConfig.Slot0 = armSlot0Configs;
        //Current Limits
        CurrentLimitsConfigs armCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.Arm.kCurrentLimitAmps)
            .withSupplyCurrentThreshold(Constants.Arm.kCurrentLimitThresholdAmps)
            .withSupplyTimeThreshold(Constants.Arm.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(Constants.Arm.kCurrentLimitEnable);
        armFXConfig.CurrentLimits = armCurrentLimitsConfigs;
        //Motion Magic
        MotionMagicConfigs armMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.Arm.kMotionMagicCruise)
            .withMotionMagicAcceleration(Constants.Arm.kMotionMagicAccel)
            .withMotionMagicJerk(Constants.Arm.kMotionMagicJerk);
        armFXConfig.MotionMagic = armMotionMagicConfigs;
        //Mechanical Limits
        SoftwareLimitSwitchConfigs armSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitEnable(Constants.Arm.kSoftReverseLimitEnable)
            .withReverseSoftLimitThreshold(Constants.Arm.kSoftReverseLimit)
            .withForwardSoftLimitEnable(Constants.Arm.kSoftForwardLimitEnable)
            .withForwardSoftLimitThreshold(Constants.Arm.kSoftForwardLimit);
        armFXConfig.SoftwareLimitSwitch = armSoftwareLimitSwitchConfigs;
        // HardwareLimitSwitchConfigs armHardwareLimitsConfigs = new HardwareLimitSwitchConfigs()
        //     .withReverseLimitEnable(false)
        //     .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
        //     .withReverseLimitAutosetPositionEnable(true)
        //     .withReverseLimitAutosetPositionValue(0.0)
        //     .withForwardLimitEnable(false)
        //     .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen); //Add autoset position on forward limit to appropriate number also.
        // armFXConfig.HardwareLimitSwitch = armHardwareLimitsConfigs;
        //Encoder
        if(Constants.Arm.kUseCANcoder) {
            armFXConfig.Feedback.FeedbackRemoteSensorID = Constants.Arm.kCANcoderID;
            armFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
            armFXConfig.Feedback.RotorToSensorRatio = Constants.Arm.kGearRatio;
            armFXConfig.Feedback.SensorToMechanismRatio = 1.0; //CANcoder is the same as mechanism
        } else {
            armFXConfig.Feedback.SensorToMechanismRatio = Constants.Arm.kGearRatio;
        }
        //Neutral and Direction
        armFXConfig.MotorOutput.NeutralMode = Constants.Arm.kNeutralMode;
        armFXConfig.MotorOutput.Inverted = (Constants.Arm.kIsInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        //Audio
        armFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);


        //Climber
        //CANcoder
        climberCCConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        climberCCConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        climberCCConfig.MagnetSensor.MagnetOffset = Constants.Climber.kMagnetOffset;

        Slot0Configs climberSlot0Configs = new Slot0Configs()
            .withKP(Constants.Climber.kP)
            .withKI(Constants.Climber.kI)
            .withKD(Constants.Climber.kD)
            .withKS(Constants.Climber.kS)
            .withKV(Constants.Climber.kV)
            .withKA(Constants.Climber.kA);
        climberFXConfig.Slot0 = climberSlot0Configs;
        //Current Limits
        CurrentLimitsConfigs climberCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.Climber.kCurrentLimitAmps)
            .withSupplyCurrentThreshold(Constants.Climber.kCurrentLimitThresholdAmps)
            .withSupplyTimeThreshold(Constants.Climber.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(Constants.Climber.kCurrentLimitEnable);
        climberFXConfig.CurrentLimits = climberCurrentLimitsConfigs;
        //Motion Magic
        MotionMagicConfigs climberMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.Climber.kMotionMagicCruise)
            .withMotionMagicAcceleration(Constants.Climber.kMotionMagicAccel)
            .withMotionMagicJerk(Constants.Climber.kMotionMagicJerk);
        climberFXConfig.MotionMagic = climberMotionMagicConfigs;
        //Mechanical Limits
        SoftwareLimitSwitchConfigs climberSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitEnable(Constants.Climber.kSoftReverseLimitEnable)
            .withReverseSoftLimitThreshold(Constants.Climber.kSoftReverseLimit)
            .withForwardSoftLimitEnable(Constants.Climber.kSoftForwardLimitEnable)
            .withForwardSoftLimitThreshold(Constants.Climber.kSoftForwardLimit);
        climberFXConfig.SoftwareLimitSwitch = climberSoftwareLimitSwitchConfigs;
        // HardwareLimitSwitchConfigs climberHardwareLimitsConfigs = new HardwareLimitSwitchConfigs()
        //     .withReverseLimitEnable(false)
        //     .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
        //     .withReverseLimitAutosetPositionEnable(true)
        //     .withReverseLimitAutosetPositionValue(0.0)
        //     .withForwardLimitEnable(false)
        //     .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen); //Add autoset position on forward limit to appropriate number also.
        // climberFXConfig.HardwareLimitSwitch = climberHardwareLimitsConfigs;
        //Encoder
        if(Constants.Aimer.kUseCANcoder) {
            climberFXConfig.Feedback.FeedbackRemoteSensorID = Constants.Climber.kCANcoderID;
            climberFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            climberFXConfig.Feedback.RotorToSensorRatio = Constants.Climber.kGearRatio;
            climberFXConfig.Feedback.SensorToMechanismRatio = Constants.Climber.kSensorGearRatio; //CANcoder is the same as mechanism
        } else {
            climberFXConfig.Feedback.SensorToMechanismRatio = Constants.Climber.kGearRatio;
        }
        //Neutral and Direction
        climberFXConfig.MotorOutput.NeutralMode = Constants.Climber.kNeutralMode;
        climberFXConfig.MotorOutput.Inverted = (Constants.Climber.kIsInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        //Audio
        climberFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);

    }

    public void retryConfigApply(Supplier<StatusCode> toApply) {
        StatusCode finalCode = StatusCode.StatusCodeNotInitialized;
        int triesLeftOver = 5;
        do{
            finalCode = toApply.get();
        } while (!finalCode.isOK() && --triesLeftOver > 0);
        assert(finalCode.isOK());
    }
}