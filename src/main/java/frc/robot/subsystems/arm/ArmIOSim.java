package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class ArmIOSim implements ArmIO {
  public SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(2),
          100,
          600,
          0.74,
          ArmConstants.MIN_ANGLE.in(Radians),
          ArmConstants.MAX_ANGLE.in(Radians),
          false,
          ArmConstants.MIN_ANGLE.in(Radians) + 0.1);

  PIDController controller = new PIDController(0, 0, 0);
  double appliedVolts = 0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    appliedVolts =
        MathUtil.clamp(controller.calculate(Units.radiansToDegrees(sim.getAngleRads())), -12, 12);
    System.out.println(appliedVolts + "  " + sim.getAngleRads());
    sim.update(0.02);
    sim.setInputVoltage(appliedVolts);
    inputs.isAtSetpoint = controller.atSetpoint();
    inputs.currentAngle = Units.radiansToDegrees(sim.getAngleRads());
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.setpoint = controller.getSetpoint();
  }

  @Override
  public void setSetpoint(Measure<Angle> setpoint) {
    controller.setSetpoint(setpoint.in(Degrees));
  }

  @Override
  public void configurePID(double kP, double kI, double kD, double setpointDeadband) {
    controller.setPID(kP, kI, kD);
    controller.setTolerance(setpointDeadband);
  }
}
