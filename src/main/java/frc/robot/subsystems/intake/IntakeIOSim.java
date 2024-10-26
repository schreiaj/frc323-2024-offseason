package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
  private final DCMotor actuationMotor = DCMotor.getNEO(1);
  //    TODO Get the actual gearing
  private final double gearing = 10.0;
  private final Distance intakeLength = Inches.of(15);
  private final Mass intakeMass = Pounds.of(3);
  private final Angle minAcuationAngle = Degrees.of(-360);
  private final Angle maxAcuationAngle = Degrees.of(360);
  private final Angle startingAngle = Degrees.of(0);
  private final SingleJointedArmSim singleJointedArmSim =
      new SingleJointedArmSim(
          actuationMotor,
          gearing,
          SingleJointedArmSim.estimateMOI(intakeMass.in(Kilograms), intakeLength.in(Meters)),
          intakeLength.in(Meters),
          minAcuationAngle.in(Radians),
          maxAcuationAngle.in(Radians),
          true,
          startingAngle.in(Radians));

  private final MutVoltage actuatorAppliedVolts = Volts.mutable(0);
  private final double actuationKS = 0.0;
  private final double actuationKG = 0.126;
  private final double actuationKV = 1.3;
  private final double actuationKA = 5;
  private final double actuationKP = 0.0;
  private final double actuationKI = 0.0;
  private final double actuationKD = 0.0;
  private final AngularVelocity actuatorMaxVelocity = DegreesPerSecond.of(360);
  private final AngularAcceleration actuatorMaxAcceleration = DegreesPerSecondPerSecond.of(360);
  private final ArmFeedforward actuationFF =
      new ArmFeedforward(actuationKS, actuationKG, actuationKV, actuationKA);
  private final ProfiledPIDController actuationController =
      new ProfiledPIDController(
          actuationKP,
          actuationKI,
          actuationKD,
          new Constraints(actuatorMaxVelocity.magnitude(), actuatorMaxAcceleration.magnitude()));

  private final DCMotor intakeMotor = DCMotor.getNEO(1);
  private  final MutVoltage intakeAppliedVolts = Volts.mutable(0);
  private final double intakeMOI = 0.1;
  private final double intakeGearing = 10.0;



  private final FlywheelSim intakeSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(intakeMotor, intakeMOI, intakeGearing), intakeMotor);

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    singleJointedArmSim.update(0.02);
    intakeSim.update(0.02);
    inputs.actuationPosition.mut_replace(singleJointedArmSim.getAngleRads(), Radians);
    inputs.actuationVelocity.mut_replace(
        singleJointedArmSim.getVelocityRadPerSec(), RadiansPerSecond);
    inputs.actuationCurrent.mut_replace(singleJointedArmSim.getCurrentDrawAmps(), Amps);
    inputs.actuationVoltage.mut_replace(actuatorAppliedVolts);
    inputs.actuationSetpoint.mut_replace(actuationController.getSetpoint().position, Degrees);
    inputs.actuationVelocity.mut_replace(
        actuationController.getSetpoint().velocity, DegreesPerSecond);

    inputs.intakeVelocity.mut_replace(intakeSim.getAngularVelocityRadPerSec(), RadiansPerSecond);
    inputs.intakeCurrent.mut_replace(intakeSim.getCurrentDrawAmps(), Amps);
    inputs.intakeVoltage.mut_replace(intakeSim.getInputVoltage(), Volts);
    }

    @Override
    public void runVolts(Voltage actuationVoltage, Voltage intakeVoltage) {
      double clampedActuationVoltage = MathUtil.clamp(actuationVoltage.in(Volts),-12, 12);
      double clampedIntakeVoltage = MathUtil.clamp(intakeVoltage.in(Volts),-12, 12);
      actuatorAppliedVolts.mut_replace(clampedActuationVoltage, Volts);
      intakeSim.setInputVoltage(clampedActuationVoltage);
      intakeAppliedVolts.mut_replace(clampedIntakeVoltage, Volts);
      intakeSim.setInputVoltage(clampedIntakeVoltage);
    }

    @Override
    public void runSetpoint(Angle actuationSetpoint, AngularVelocity intakeSetpoint) {
      Angle currentActuationAngle = Radians.of(singleJointedArmSim.getAngleRads());

      Angle actuationSetpointAngle = Degrees.of(actuationController.getSetpoint().position);
      AngularVelocity actuationSetpointVelocity = DegreesPerSecond.of(actuationController.getSetpoint().velocity);

    }


}
