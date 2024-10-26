package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;
import static edu.wpi.first.units.Units.*;

public interface IntakeIO {

    @AutoLog
    class IntakeIOInputs {
        public boolean actuationMotorConnected, intakeMotorConnected = true;

        // Actuation motor fields
        public MutAngle actuationPosition = Degrees.mutable(0);
        public MutAngularVelocity actuationVelocity = RPM.mutable(0);
        public MutCurrent actuationCurrent = Amps.mutable(0);
        public MutTemperature actuationTemperature = Celsius.mutable(0);
        public MutVoltage actuationVoltage = Volts.mutable(0);
        public MutAngle actuationSetpoint = Degrees.mutable(0);

        // Intake motor fields
        public MutAngularVelocity intakeVelocity = RPM.mutable(0);
        public MutCurrent intakeCurrent = Amps.mutable(0);
        public MutTemperature intakeTemperature = Celsius.mutable(0);
        public MutVoltage intakeVoltage = Volts.mutable(0);
        public MutAngularVelocity intakeSetpoint = RPM.mutable(0);
    }

    default void updateInputs(IntakeIOInputs inputs) {}
    default void runVolts(Voltage actuationVoltage, Voltage intakeVoltage) {}
    default void runCurrent(Current actuationCurrent, Current intakeCurrent) {}

    default void runSetpoint(Angle actuationSetpoint, AngularVelocity intakeSetpoint) {}

    default void setActuationPID(double kP, double kI, double kD) {}
    default void setIntakePID(double kP, double kI, double kD) {}
    default void setActuationBrakeMode(boolean enable) {}
    default void setIntakeBrakeMode(boolean enable) {}


    default void stop() {}
}