// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants.SwerveModuleConstants;

/** Add your docs here. */
public class SwerveModuleIOMapleSim implements SwerveModuleIO {

    // reference to module simulation
    private final SwerveModuleSimulation moduleSimulation;
    // reference to the simulated drive motor
    private final SimulatedMotorController.GenericMotorController driveMotor;
    // reference to the simulated turn motor
    private final SimulatedMotorController.GenericMotorController steerMotor;

    private final double steerkS = 0.2;
    private final double steerkPVoltsPerRotError = 34.0;

    private SimpleMotorFeedforward feedForwardSteer = new SimpleMotorFeedforward(steerkS, 0.0);
    private PIDController steerPID = new PIDController(steerkPVoltsPerRotError, 0.0, 0.0);

    private final double drivekS = 0.25; 
    private final double drivekVVoltsPerRps = 0.872844827583;// 2.1volts - 0.8 mps
    private final double drivekPVoltsPerRpsError = 0.2;

    private SimpleMotorFeedforward feedForwardDrive = new SimpleMotorFeedforward(drivekS, drivekVVoltsPerRps);

    public SwerveModuleIOMapleSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;

        // configures a generic motor controller for drive motor
        // set a current limit of 60 amps
        this.driveMotor = moduleSimulation
                .useGenericMotorControllerForDrive()
                .withCurrentLimit(Current.ofRelativeUnits(40, Amps));
        this.steerMotor = moduleSimulation
                .useGenericControllerForSteer()
                .withCurrentLimit(Current.ofRelativeUnits(45, Amps));
    }

    @Override
    public void setDriveVelocity(double velocityMetersPerSecond) {
        double velocityRotationsPerSecondDriveWheels = velocityMetersPerSecond/SwerveModuleConstants.wheelCircumferenceMeters;
        double velocityRotationsPerSecondDriveMotor = velocityRotationsPerSecondDriveWheels/SwerveModuleConstants.driveGearReduction;
        
        double outputVolts = feedForwardDrive.calculate(velocityRotationsPerSecondDriveMotor);
        setDriveVoltage(outputVolts);
    }

    @Override
    public void setTurnAngle(double angleDegrees) {
        double errorRotations = Units.degreesToRotations(angleDegrees - this.moduleSimulation.getSteerAbsoluteAngle().in(Degrees));
        double outputVoltage = steerPID.calculate(errorRotations) + feedForwardSteer.calculate(angleDegrees);
        setAngleVoltage(outputVoltage*-1.0);
    }

    @Override
    public void setDriveVoltage(double voltage) {
        this.driveMotor.requestVoltage(Volts.of(voltage));
    }

    @Override
    public void setAngleVoltage(double voltage) {
        this.steerMotor.requestVoltage(Volts.of(voltage));
    }

    
    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.angleAbsolutePositionDegrees = this.moduleSimulation.getSteerAbsoluteAngle().in(Degrees);
        inputs.driveAppliedVoltage = this.moduleSimulation.getDriveMotorAppliedVoltage().in(Volts);
        inputs.driveCurrent = this.moduleSimulation.getDriveMotorStatorCurrent().in(Amps);
        inputs.drivePositionMeters = this.moduleSimulation.getDriveWheelFinalPosition().in(Rotation)
            *SwerveModuleConstants.wheelCircumferenceMeters;
        inputs.driveVelocityMetersPerSecond = this.moduleSimulation.getDriveWheelFinalSpeed().in(RotationsPerSecond)
            *SwerveModuleConstants.wheelCircumferenceMeters;
    }
}
