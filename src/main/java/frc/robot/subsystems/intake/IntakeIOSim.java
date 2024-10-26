package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
  private FlywheelSim frontMotorSim =
      new FlywheelSim(DCMotor.getNEO(1), IntakeConstants.FRONT_MOTOR_GEAR_RATIO, 0.004);
  private FlywheelSim rearMotorSim =
      new FlywheelSim(DCMotor.getNEO(1), IntakeConstants.FRONT_MOTOR_GEAR_RATIO, 0.004);
  private double frontMotorAppliedVolts = 0.0;
  private double rearMotorAppliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.frontMotorVelocityRotationsPerMinute = frontMotorSim.getAngularVelocityRPM();
    inputs.rearMotorVelocityRotationsPerMinute = rearMotorSim.getAngularVelocityRPM();
    inputs.frontMotorAppliedVolts = frontMotorAppliedVolts;
    inputs.rearMotorAppliedVolts = rearMotorAppliedVolts;
    inputs.currentAmps =
        new double[] {frontMotorSim.getCurrentDrawAmps(), rearMotorSim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    frontMotorAppliedVolts = volts;
    rearMotorAppliedVolts = volts;
    frontMotorSim.setInputVoltage(volts);
    rearMotorSim.setInputVoltage(volts);
  }
}
