// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.TurnToAngle;
import frc.robot.lib.util.LED;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.DriveForDistance;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootBallSubsystems;
import frc.robot.commands.ShooterPID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Feeder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Swerve swerveDrivetrain = new Swerve(true);
  XboxController driver = new XboxController(0);
  SwerveDriveCommand driveCommand = new SwerveDriveCommand(swerveDrivetrain, driver);
  LEDSubsystem LED = new LEDSubsystem();

  
  public Shooter shooter = new Shooter();
  public ShooterPID shooterpid = new ShooterPID(shooter, 2000);
  //AutoShoot autoShoot = new AutoShoot(shooter, swerveDrivetrain);

  public Intake intake = new Intake();
  public IntakeCommand intakeCommand = new IntakeCommand(intake);

  public Storage storage = new Storage();

  public Feeder feeder = new Feeder(false);
  
  public TurnToAngle turnToAngle = new TurnToAngle(swerveDrivetrain, 45);

  public ShootBallSubsystems shootBallSubsystems = new ShootBallSubsystems(shooter, feeder, storage);

  public DriveForDistance driveForDistance = new DriveForDistance(swerveDrivetrain, 3);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
    // commented out because shooter is broken
    swerveDrivetrain.setDefaultCommand(driveCommand);

  
    /*JoystickButton ledButton = new JoystickButton(driver, 1);

    ledButton.whenPressed(new RunCommand(()-> LED.turnOn(), LED));
    ledButton.whenReleased(
      new RunCommand(
        () -> LED.turnOff(),
        LED)
    );
    new JoystickButton(driver,2).whenHeld(shooterpid);*/

    //Storage Feed balls
    new JoystickButton(driver, 4).whileHeld(new RunCommand(()-> storage.bothBackward(), storage));
    new JoystickButton(driver, 4).whenReleased(new RunCommand(()-> storage.stop(), storage));
    //Storage Reverse
    new JoystickButton(driver, 3).whileHeld(new RunCommand(()->storage.bothForward(), storage));
    new JoystickButton(driver,3).whenReleased(new RunCommand(()->storage.stop(), storage));
    
    JoystickButton driveButton = new JoystickButton(driver, 1);
    driveButton.whileHeld(turnToAngle);
    driveButton.whenReleased(
    new RunCommand(
      () ->  swerveDrivetrain.drive(0,0,0, false),
      swerveDrivetrain));
    

    //TURN TO ANGLE
    //new JoystickButton(driver, 1).whileHeld(turnToAngle);

    JoystickButton feederButton = new JoystickButton(driver, 8);
    feederButton.whileHeld(new RunCommand(()-> feeder.runForward(), feeder));
    feederButton.whenReleased(new RunCommand(()-> feeder.stop(), feeder));

    JoystickButton shooterButton = new JoystickButton(driver, 2);
    shooterButton.whileHeld(shooterpid);
    
    

    //TODO SHOOTER SUBSYSTEM BUTTON BINDING



  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return intakeCommand;
  }
}
