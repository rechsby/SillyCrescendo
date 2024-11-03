package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

public class ModuleIOSimMaple implements ModuleIO {
  private final SwerveModuleSimulation moduleSimulation;

  public ModuleIOSimMaple(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPositionRad();
    inputs.driveVelocityRadPerSec = moduleSimulation.getDriveWheelFinalSpeedRadPerSec();
    inputs.driveAppliedVolts = Math.abs(moduleSimulation.getDriveMotorAppliedVolts());
    inputs.driveCurrentAmps = new double[] {moduleSimulation.getDriveMotorSupplyCurrentAmps()};

    inputs.turnAbsolutePosition = moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnPosition =
        Rotation2d.fromRadians(moduleSimulation.getSteerRelativeEncoderPositionRad());
    inputs.turnVelocityRadPerSec = moduleSimulation.getSteerAbsoluteEncoderSpeedRadPerSec();
    inputs.turnAppliedVolts = Math.abs(moduleSimulation.getSteerMotorAppliedVolts());
    inputs.turnCurrentAmps = new double[] {moduleSimulation.getSteerMotorSupplyCurrentAmps()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    moduleSimulation.requestDriveVoltageOut(MathUtil.clamp(volts, -12.0, 12.0));
  }

  @Override
  public void setTurnVoltage(double volts) {
    moduleSimulation.requestSteerVoltageOut(MathUtil.clamp(volts, -12.0, 12.0));
  }
}

// public interface ModuleIO {
//     @AutoLog
//     public static class ModuleIOInputs {
//       public double drivePositionRad = 0.0;
//       public double driveVelocityRadPerSec = 0.0;
//       public double driveAppliedVolts = 0.0;
//       public double[] driveCurrentAmps = new double[] {};

//       public Rotation2d turnAbsolutePosition = new Rotation2d();
//       public Rotation2d turnPosition = new Rotation2d();
//       public double turnVelocityRadPerSec = 0.0;
//       public double turnAppliedVolts = 0.0;
//       public double[] turnCurrentAmps = new double[] {};
//     }

//     /** Updates the set of loggable inputs. */
//     public default void updateInputs(ModuleIOInputs inputs) {}

//     /** Run the drive motor at the specified voltage. */
//     public default void setDriveVoltage(double volts) {}

//     /** Run the turn motor at the specified voltage. */
//     public default void setTurnVoltage(double volts) {}

//     /** Enable or disable brake mode on the drive motor. */
//     public default void setDriveBrakeMode(boolean enable) {}

//     /** Enable or disable brake mode on the turn motor. */
//     public default void setTurnBrakeMode(boolean enable) {}
//   }
