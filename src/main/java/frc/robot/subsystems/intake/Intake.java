package frc.robot.subsystems.intake;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    IntakeIOInputsAutoLogged inputs;
    IntakeIO io;

    boolean isIntakeDown = false;

    public Intake(IntakeIO io) {
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("intakeInputs", inputs);
        isIntakeDown = inputs.intakePositionDegrees < 15.0;
    }

    public boolean isIntakeDown() {
        return isIntakeDown;
    }

    public void intakeRunRollers() {
        io.setTargetRollerBottomVelocity(75.0);
        io.setTargetRollerTopVelocity(75.0);
    }

    public void intakeRollersStop() {
        io.setTargetRollerBottomVelocity(0.0);
        io.setTargetRollerTopVelocity(0.0);
    }

    public void reverseIntake() {
        io.setTargetRollerBottomVelocity(-40.0);
        io.setTargetRollerTopVelocity(-40.0);
    }

    public void intakeDown() {
        intakeRollersStop();
        if(inputs.intakePositionDegrees > 15.0) {
            isIntakeDown = false;
            io.setTargetIntakePositionDegrees(0); 
        } else {
            isIntakeDown = true;
            intakeDefault();
        }
    }

    public void intakeDownThenIntake() {
        // if(inputs.intakePositionDegrees > 7.5) {
        //     isIntakeDown = false;
        //     io.setTargetIntakePositionDegrees(0);
        //     // reverseIntake();
        // } else {
        //     isIntakeDown = true;
        //     intakeDefault();
        // }

        io.setTargetIntakePositionDegrees(0);

        // if(inputs.intakePositionDegrees < 40.0) {
            
        // }
        intakeRunRollers();
    }

    public void intakeDefualtAndIntake() {
        io.setIntakeVolts(0);
        intakeRunRollers();
    }

    public void intakeUp() {
        io.setTargetIntakePositionDegrees(IntakeConstants.intakeUpDegrees);
    }

    public void setIntakeDegEncoder(double deg) {
        io.setIntakePositionEncoder(deg);
    }

    public void intakeUpAndIntake() {
        intakeUp();
        if(inputs.intakePositionDegrees < 19.0) {
            io.setTargetRollerBottomVelocity(15.0);
            io.setTargetRollerTopVelocity(15.0);
            // io.setRollerTopVolts(0.0);
            // io.setRollerBottomVolts(0.0);
        } else {
            io.setRollerTopVolts(0.0);
            io.setRollerBottomVolts(0.0);
        }
        // intakeRunRollers();
    }

    public void intakeDefault() {
        io.setIntakeVolts(0);
    }

    public void rollerAndIntakeNoVolts() {
        intakeDefault();
        intakeRollersStop();
    }

    public void setAllVolts(double topRollerVolts, double bottomRollerVolts, double pivotVolts) {
        io.setRollerTopVolts(topRollerVolts);
        io.setRollerBottomVolts(bottomRollerVolts);
        io.setIntakeVolts(pivotVolts);
    }

    public Command noVoltageCommand() {
        return this.run(() -> rollerAndIntakeNoVolts());
    }

    public Command intakeDownCommand() {
        return this.run(() -> intakeDown());
    }

    public Command zeroIntakeCommand(double angleDeg) {
        return this.run(() -> setIntakeDegEncoder(angleDeg));
    }


    
    public Command intakeUpCommand() {
        return this.run(() -> intakeUp());
    }

    public Command intakeRunRollersCommand() {
        return this.run(() -> intakeRunRollers());
    }

    public Command reverseIntakeCommand() {
        return this.run(() -> reverseIntake());
    }

    public Command intakeDownThenIntakeCommand() {
        return this.run(() -> intakeDownThenIntake());
    }

    public Command intakeDefualtAndIntakeCommand() {
        return this.run(() -> intakeDefualtAndIntake());
    }

    public Command setAllVoltsCommand(Supplier<Double> topRollerVolts, Supplier<Double> bottomRollerVolts, Supplier<Double> pivotVolts) {
        return this.run(() -> setAllVolts(topRollerVolts.get(), bottomRollerVolts.get(), pivotVolts.get()));
    }

}

