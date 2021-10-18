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
import frc.robot.lib.util.LED;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterPID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Swerve swerveDrivetrain = new Swerve(false);
  Joystick driver = new Joystick(0);
  SwerveDriveCommand driveCommand = new SwerveDriveCommand(swerveDrivetrain, driver);
  LEDSubsystem LED = new LEDSubsystem();

  
  public Shooter shooter = new Shooter();
  public ShooterPID shooterpid = new ShooterPID(shooter);
  AutoShoot autoShoot = new AutoShoot(LED, shooter, swerveDrivetrain);

  public Intake intake = new Intake();
  public IntakeCommand intakeCommand = new IntakeCommand(intake);

  public Storage storage = new Storage();


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
    // new JoystickButton(driver, 3).whileHeld(shooterpid);
    // ! Commented for shooter test swerveDrivetrain.setDefaultCommand(driveCommand);

  
    /*JoystickButton ledButton = new JoystickButton(driver, 1);

    ledButton.whenPressed(new RunCommand(()-> LED.turnOn(), LED));
    ledButton.whenReleased(
      new RunCommand(
        () -> LED.turnOff(),
        LED)
    );
    new JoystickButton(driver,2).whenHeld(shooterpid);*/

    //Storage Feed balls
    new JoystickButton(driver, 3).whileHeld(new RunCommand(()-> storage.bothBackward(), storage));
    new JoystickButton(driver, 3).whenReleased(new RunCommand(()-> storage.stop(), storage));

    //Storage Reverse
    new JoystickButton(driver, 4).whileHeld(new RunCommand(()->storage.bothForward(), storage));
    new JoystickButton(driver,4).whenReleased(new RunCommand(()->storage.stop(), storage));
    new JoystickButton(driver, 1).whileHeld(intakeCommand);

  
   
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
