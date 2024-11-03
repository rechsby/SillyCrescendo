// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSimMaple;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.transport.TransportIO;
import frc.robot.subsystems.transport.TransportIOSim;
import frc.robot.subsystems.transport.TransportIOSparkMax;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation.DRIVE_WHEEL_TYPE;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final GyroSimulation gyroSimulation;
  public final SwerveDriveSimulation swerveDriveSimulation;
  private final Drive drive;

  private static Flywheel flywheel;
  private static Intake intake;
  private static Transport transport;
  private static Shooter shooter;
  private static Arm arm;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        gyroSimulation = null;
        swerveDriveSimulation = null;
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
        flywheel = new Flywheel(new FlywheelIOSparkMax());
        intake = new Intake((new IntakeIOSparkMax()));
        transport = new Transport(new TransportIOSparkMax());
        arm = new Arm(new ArmIOSparkMax());
        // shooter = new Shooter(new ShooterIOSparkMax());
        // drive = new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFX(0),
        // new ModuleIOTalonFX(1),
        // new ModuleIOTalonFX(2),
        // new ModuleIOTalonFX(3));
        // flywheel = new Flywheel(new FlywheelIOTalonFX());
        break;

      case SIM:
        gyroSimulation = GyroSimulation.createPigeon2();
        this.swerveDriveSimulation =
            new SwerveDriveSimulation(
                49.8,
                0.65,
                0.65,
                0.74,
                0.74,
                SwerveModuleSimulation.getMark4i(
                    DCMotor.getFalcon500(1), DCMotor.getNEO(1), 80, DRIVE_WHEEL_TYPE.TIRE, 1),
                gyroSimulation,
                new Pose2d(3, 2, new Rotation2d()));

        // Sim robot, instantiate physics sim IO implementations
        // drive =
        //     new Drive(
        //         new GyroIO() {},
        //         new ModuleIOSim(),
        //         new ModuleIOSim(),
        //         new ModuleIOSim(),
        //         new ModuleIOSim());
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
        this.drive =
            new Drive(
                new GyroIOSim(this.gyroSimulation),
                new ModuleIOSimMaple(this.swerveDriveSimulation.getModules()[0]),
                new ModuleIOSimMaple(this.swerveDriveSimulation.getModules()[1]),
                new ModuleIOSimMaple(this.swerveDriveSimulation.getModules()[2]),
                new ModuleIOSimMaple(this.swerveDriveSimulation.getModules()[3]));
        this.drive.setPose(new Pose2d(3, 2, new Rotation2d()));

        SimulatedArena.getInstance().resetFieldForAuto();

        flywheel = new Flywheel(new FlywheelIOSim());
        intake = new Intake(new IntakeIOSim());
        transport = new Transport(new TransportIOSim());
        shooter = new Shooter(new ShooterIOSim());
        arm = new Arm(new ArmIOSim());
        break;

      default:
        gyroSimulation = null;
        swerveDriveSimulation = null;
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        intake = new Intake(new IntakeIO() {});
        transport = new Transport(new TransportIO() {});
        shooter = new Shooter(new ShooterIO() {});
        arm = new Arm(new ArmIO() {});
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(
                () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
            .withTimeout(5.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Forward)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Reverse)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Forward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Reverse)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -MathUtil.applyDeadband(controller.getLeftY(), 0.1),
            () -> -MathUtil.applyDeadband(controller.getLeftX(), 0.1),
            () -> -MathUtil.applyDeadband(controller.getRightX(), 0.1)));

    controller
        .leftTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  intake.intake();
                },
                RobotContainer.intake))
        .onFalse(
            Commands.runOnce(
                () -> {
                  intake.stop();
                },
                RobotContainer.intake));

    controller
        .rightTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.runVelocity(3500);
                },
                RobotContainer.shooter));

    controller
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  arm.setSetpoint(Degrees.of(50));
                },
                RobotContainer.arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void updateSimulationField() {
    if (swerveDriveSimulation != null) {
      SimulatedArena.getInstance().simulationPeriodic();

      Logger.recordOutput(
          "FieldSimulation/RobotPosition", swerveDriveSimulation.getSimulatedDriveTrainPose());

      final List<Pose3d> notes = SimulatedArena.getInstance().getGamePiecesByType("Note");
      if (notes != null) Logger.recordOutput("FieldSimulation/Notes", notes.toArray(Pose3d[]::new));
    }
  }
}
