package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
    switch (Constants.currentMode) {
      case REAL:
        io.configureTopMotorPID(1.932E-09, 0.0, 0.0, ShooterConstants.RPM_DEADBAND);
        io.configureBottomMotorPID(8.0536E-09, 0.0, 0.0, ShooterConstants.RPM_DEADBAND);
        break;
      case REPLAY:
        io.configureTopMotorPID(1.0, 0.0, 0.0, ShooterConstants.RPM_DEADBAND);
        io.configureBottomMotorPID(1.0, 0.0, 0.0, ShooterConstants.RPM_DEADBAND);
        break;
      case SIM:
        io.configureTopMotorPID(1.0, 0.0, 0.0, ShooterConstants.RPM_DEADBAND);
        io.configureBottomMotorPID(1.0, 0.0, 0.0, ShooterConstants.RPM_DEADBAND);
        break;
      default:
        io.configureTopMotorPID(1.0, 0.0, 0.0, ShooterConstants.RPM_DEADBAND);
        io.configureBottomMotorPID(1.0, 0.0, 0.0, ShooterConstants.RPM_DEADBAND);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setTopMotorVoltage(volts);
    io.setBottomMotorVelocity(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runTopMotorVelocity(double velocityRPM) {
    io.setTopMotorVelocity(velocityRPM);
    Logger.recordOutput("Shooter/TopMotorSetpointRPM", velocityRPM);
  }

  /** Run closed loop at the specified velocity. */
  public void runBottomMotorVelocity(double velocityRPM) {
    io.setBottomMotorVelocity(velocityRPM);
    Logger.recordOutput("Shooter/BottomMotorSetpointRPM", velocityRPM);
  }

  public void runVelocity(double velocityRPM) {
    runTopMotorVelocity(velocityRPM);
    runBottomMotorVelocity(velocityRPM);
  }

  public boolean isAtSetpoint() {
    return inputs.isAtSetpoint;
  }
  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }
}
