package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.VendorWrappers.Kraken;

public class SwerveModuleIOKraken implements SwerveModuleIO {

    private double desiredAngleDeg = 0.0;
    private Orchestra orchestra;

    final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(true)
        .withUpdateFreqHz(60.0);
    final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(1).withEnableFOC(true)
        .withUpdateFreqHz(60.0);

    private CANcoder absoluteEncoder;
    private Kraken angleMotor;
    private Kraken driveMotor;


    public SwerveModuleIOKraken(int driveMotorID, int angleMotorID, double angleOffsetRotations, int cancoderID, String name, boolean isInverted) {
        this(driveMotorID, angleMotorID, angleOffsetRotations, cancoderID, isInverted, false, name);
    }

    /**
     * 
     * @param driveMotorID - ID of the drive motor
     * @param angleMotorID - ID of the angle motor
     * @param angleOffsetRotations - Offset of the angle motor, in degrees
     * @param cancoderID - ID of the absolute CANcoder mounted ontop of the swerve
     * @param isDriveMotorOnTop - Is drive motor mounted on top
     * @param isAngleMotorOnTop - Is angle motor mounted on top
     */
    public SwerveModuleIOKraken(int driveMotorID, int angleMotorID, double angleOffsetRotations, int cancoderID, boolean isDriveMotorOnTop, boolean isAngleMotorOnTop, String name){
        /* Angle Encoder Config */
        absoluteEncoder = new CANcoder(cancoderID, UniversalConstants.canivoreName);
        configCANCoder(angleOffsetRotations);

        /* Angle Motor Config */
        angleMotor = new Kraken(name+"Steer", angleMotorID, UniversalConstants.canivoreName);
        if(isAngleMotorOnTop) {
            configAngleMotor(InvertedValue.Clockwise_Positive,cancoderID);
        } else {
            configAngleMotor(InvertedValue.CounterClockwise_Positive,cancoderID);
        }

        /* Drive Motor Config */
        driveMotor = new Kraken(name+"Drive", driveMotorID, UniversalConstants.canivoreName);
        if(isDriveMotorOnTop) {
            configDriveMotor(InvertedValue.CounterClockwise_Positive);
        } else {
            configDriveMotor(InvertedValue.Clockwise_Positive);
        }
        orchestra = new Orchestra();
        orchestra.addInstrument(driveMotor);
        orchestra.addInstrument(angleMotor);
    }

    private void configCANCoder(double angleOffsetRotations) {
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cancoderConfigs.MagnetSensor.MagnetOffset = angleOffsetRotations;
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        absoluteEncoder.getConfigurator().apply(cancoderConfigs);
    }

    private void configDriveMotor(InvertedValue invertedValue) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = invertedValue;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 40; // re-determined after firmware upgrade to prevent wheel slip. Feels pretty low though

        config.Slot1.kS = 0.25; 
        config.Slot1.kV = 0.872844827583;// 2.1volts - 0.8 mps
        config.Slot1.kP = 0.2;
        config.Slot1.kI = 0.0;
        config.Slot1.kD = 0.0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = 1.0/SwerveModuleConstants.driveGearReduction;  //stage 2 is 6.03:1, stage 1 is 5.27:1, 
        config.ClosedLoopGeneral.ContinuousWrap = true;
        driveMotor.applyConfig(config);
    }

    private void configAngleMotor(InvertedValue invertedValue, int cancoderID) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = invertedValue;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.StatorCurrentLimit = 45;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 0.2;
        config.Slot0.kP = 34.0;
        config.Slot0.kI = 0.0; 
        config.Slot0.kD = 0.0;
        config.Feedback.FeedbackRemoteSensorID = cancoderID;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.SensorToMechanismRatio = 1;
        config.ClosedLoopGeneral.ContinuousWrap = true;
        // config.Feedback.RotorToSensorRatio = 26.09;
        angleMotor.applyConfig(config);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionMeters = driveMotor.getPosition().getValueAsDouble() * (SwerveModuleConstants.wheelCircumferenceMeters);
        inputs.driveVelocityMetersPerSecond = driveMotor.getVelocity().getValueAsDouble() * (SwerveModuleConstants.wheelCircumferenceMeters);
        inputs.angleAbsolutePositionDegrees = absoluteEncoder.getAbsolutePosition().getValueAsDouble()*360;
        inputs.targetAngleDegrees = desiredAngleDeg;

        inputs.driveAppliedVoltage = driveMotor.getMotorVoltage().getValueAsDouble();
        inputs.driveCurrent = driveMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setDriveVelocity(double velocityMetersPerSecond) {
        double velocityRotationsPerSecondDriveWheels = velocityMetersPerSecond/SwerveModuleConstants.wheelCircumferenceMeters;
        // double velocityRotationsPerSecondDriveMotor = velocityRotationsPerSecondDriveWheels/SwerveModuleConstants.driveGearReduction;
        driveMotor.setControl(velocityRequest.withVelocity(velocityRotationsPerSecondDriveWheels));
    }

    @Override
    public void setTurnAngle(double angleDegrees) {
        desiredAngleDeg = angleDegrees;
        angleMotor.setControl(positionRequest.withPosition(Units.degreesToRotations(angleDegrees)));
    }
    
    
    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setAngleVoltage(double volts) {
        angleMotor.setVoltage(volts);
    }
    
    @Override
    public void playMusic(String songName) {
        orchestra.loadMusic(songName);
        orchestra.play();
    }

    @Override
    public void stopMusic() {
        orchestra.stop();
    }
}
