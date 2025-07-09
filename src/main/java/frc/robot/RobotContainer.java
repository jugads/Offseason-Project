// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ClimberConstants.*;
import frc.robot.Constants.ElevatorConstants.*;
import frc.robot.Constants.HopperConstants.*;
import frc.robot.Superstructure.WantedSuperState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Arm.ArmIOSparkMax;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Arm.ArmSubsystem.WantedState;
import frc.robot.subsystems.Bluetooth.BluetoothIOSparkMax;
import frc.robot.subsystems.Bluetooth.BluetoothSubsystem;
import frc.robot.subsystems.Climber.ClimberIOSparkMax;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Hopper.HopperIO;
import frc.robot.subsystems.Hopper.HopperIOSparkMax;
import frc.robot.subsystems.Hopper.HopperSubsystem;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  HopperSubsystem hopper;
  Superstructure superstructure;
  ElevatorSubsystem elevator;
  ClimberSubsystem climber;
  ArmSubsystem arm;
  BluetoothSubsystem bluetooth;
  CommandSwerveDrivetrain drivetrain;
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = 3 * Math.PI;
    private double gyro = 1;
    // Swerve drive requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed*0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric driveRR = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final CommandXboxController m_driverController =
      new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    hopper = new HopperSubsystem(new HopperIOSparkMax(6,7));
    elevator = new ElevatorSubsystem(new ElevatorIOSparkMax(1, 2));
    climber = new ClimberSubsystem(new ClimberIOSparkMax(8, 9));
    arm = new ArmSubsystem(new ArmIOSparkMax(3));
    bluetooth = new BluetoothSubsystem(new BluetoothIOSparkMax(5));
    drivetrain = TunerConstants.createDrivetrain();
    superstructure = new Superstructure(hopper, elevator, climber);

    drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                .withVelocityX(-m_driverController.getLeftY() * MaxSpeed *gyro)
                .withVelocityY(-m_driverController.getLeftX() * MaxSpeed * gyro)
                .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)
            )
        );
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.a().onTrue(
      arm.setWantedStateCommand(ArmSubsystem.WantedState.GO_TO_REGULAR_SCORE)
    )
    .onFalse(arm.setWantedStateCommand(ArmSubsystem.WantedState.HOLD));

    m_driverController.y().onTrue(
      bluetooth.setWantedStateCommand(BluetoothSubsystem.WantedState.SUCKING)
    )
    .onFalse(bluetooth.setWantedStateCommand(BluetoothSubsystem.WantedState.IDLE));

    m_driverController.leftBumper().onTrue(
      Commands.sequence(
        bluetooth.setWantedStateCommand(BluetoothSubsystem.WantedState.SCORING),
        new WaitUntilCommand(() -> !bluetooth.hasCoral()),
        bluetooth.setWantedStateCommand(BluetoothSubsystem.WantedState.IDLE)
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.none();
  }
}
