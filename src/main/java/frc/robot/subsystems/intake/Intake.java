package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private Angle actuationSetpoint = Degrees.of(0);
    private AngularVelocity intakeSetpoint = RPM.of(0);

    public Intake(IntakeIO io) {
        this.io = io;
//        TODO: These are just placeholder values
        this.io.setActuationPID(.15, 0, 0);
    }

    @Override
    public void periodic() {
     super.periodic();

     if(edu.wpi.first.wpilibj.RobotState.isDisabled()){
         this.io.stop();
     }
     else {
         this.io.updateInputs(this.inputs);
         Logger.processInputs("Intake", inputs);
         this.io.runSetpoint(this.actuationSetpoint, this.intakeSetpoint);
     }
    }

    public Command setPosition(Angle position) {
        return runOnce(() -> this.actuationSetpoint = position);
    }

    public Command setVelocity(AngularVelocity velocity) {
        return runOnce(() -> this.intakeSetpoint = velocity);
    }

}
