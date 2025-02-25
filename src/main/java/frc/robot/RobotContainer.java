// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.IntakeIR;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HorizontalElevatorSubsystem;
import frc.robot.subsystems.NewIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private String selectedAuto;
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  public static final SendableChooser<String> songChooser = new SendableChooser<>();

  
  // The robot's subsystems
  public static final IntakeIR D_INTAKE_IR = new IntakeIR(0);
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(D_INTAKE_IR);
  public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final HorizontalElevatorSubsystem horizontalElevatorSubsystem = new HorizontalElevatorSubsystem();
  public static final NewIntakeSubsystem newIntakeSubsystem = new NewIntakeSubsystem();

  

  public boolean isInCoralMode = true;


  // The driver's controller
  public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  Trigger rTButton;
  Trigger lTButton;


  // robot state boolean suppliers
public static BooleanSupplier isNotAlignedRight() {
    Pose3d targetingYSpeed = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
    return (BooleanSupplier) () -> {
        if (Math.abs(targetingYSpeed.getX()+DriveSubsystem.autoAlignYoffsetRight) > 0.04
        &&  (Math.abs(LimelightHelpers.getTY("limelight")+DriveSubsystem.autoAlignXoffset)) > 0.3 ) {
            return true;
        } else return false;
    };
}
public static BooleanSupplier isNotAlignedLeft() {
  Pose3d targetingYSpeed = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
  return (BooleanSupplier) () -> {
      if (Math.abs(targetingYSpeed.getX()+DriveSubsystem.autoAlignYoffsetLeft) > 0.04
      &&  (Math.abs(LimelightHelpers.getTY("limelight")+DriveSubsystem.autoAlignXoffset)) > 0.3 ) {
          return true;
      } else return false;
  };
}
public static BooleanSupplier isNotAlignedMiddle() {
  Pose3d targetingYSpeed = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
  return (BooleanSupplier) () -> {
      if (Math.abs(targetingYSpeed.getX()) > 0.04
      &&  (Math.abs(LimelightHelpers.getTY("limelight")+DriveSubsystem.autoAlignXoffset)) > 0.3 ) {
          return true;
      } else return false;
  };
}
public static BooleanSupplier isNotAlignedFar() {
  Pose3d targetingYSpeed = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
  return (BooleanSupplier) () -> {
      if (Math.abs(targetingYSpeed.getX()) > 0.04
      &&  (Math.abs(LimelightHelpers.getTY("limelight")+DriveSubsystem.autoAlignXoffsetFar)) > 0.3 ) {
          return true;
      } else return false;
  };
}

//led's are connected to 17


public static BooleanSupplier isRed(){
  Optional<Alliance> ally = DriverStation.getAlliance();
  return (BooleanSupplier) () -> {
    if (ally.get() == Alliance.Red){
      return true;
    } else return false;
};
}

public static BooleanSupplier isInIntakeZone(){

  LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
  return (BooleanSupplier) () -> {
    if ((mt2.pose.getX()<10 || mt2.pose.getX()>40) && (mt2.pose.getY() < 10 || mt2.pose.getY()>40)){
      return true;
    } else return false;
    
   
};
}


// LimelightHelpers.getTY("limelight")+DriveSubsystem.autoAlignXoffset) < 0.01
// Math.abs(targetingYSpeed.getX()+DriveSubsystem.autoAlignYoffsetRight) > 0.04
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();
    autoChooser.addOption("FarSide3Coral", "FarSide3Coral");
    autoChooser.addOption("Move", "MoveFromZone");
    SmartDashboard.putData("Autos", autoChooser);
    

    //ADD DIFFERENT AUTOS
    //autoChooser.addOption("Complex Auto", m_complexAuto);
    
    

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY() * OIConstants.driveSensitivity, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX() * OIConstants.driveSensitivity, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX() * OIConstants.driveSensitivity, OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
    //pathplannerL4commands
    NamedCommands.registerCommand("RightL4Score", m_robotDrive.AutoAlignRight().andThen(elevatorSubsystem.raiseElevatorL4().alongWith(horizontalElevatorSubsystem.HElevatorForwardWithL4Clearance())));
    NamedCommands.registerCommand("LeftL4Score", m_robotDrive.AutoAlignLeft().andThen(elevatorSubsystem.raiseElevatorL4().alongWith(horizontalElevatorSubsystem.HElevatorForwardWithL4Clearance())));
    //pathplannerL3Commands
    NamedCommands.registerCommand("RightL3Score", m_robotDrive.AutoAlignRight().andThen(elevatorSubsystem.raiseElevatorL3().alongWith(horizontalElevatorSubsystem.HElevatorForwardWithL3Clearance())));
    NamedCommands.registerCommand("LeftL3Score", m_robotDrive.AutoAlignLeft().andThen(elevatorSubsystem.raiseElevatorL3().alongWith(horizontalElevatorSubsystem.HElevatorForwardWithL3Clearance())));
    //pathplnnerL2Commands
    NamedCommands.registerCommand("RightL2Score", m_robotDrive.AutoAlignRight().andThen(elevatorSubsystem.raiseElevatorL2().alongWith(horizontalElevatorSubsystem.HElevatorForwardWithL2Clearance())));
    NamedCommands.registerCommand("LeftL2Score", m_robotDrive.AutoAlignLeft().andThen(elevatorSubsystem.raiseElevatorL2().alongWith(horizontalElevatorSubsystem.HElevatorForwardWithL2Clearance())));
    //intake
    NamedCommands.registerCommand("Intake",newIntakeSubsystem.IntakeCoralSubstation().alongWith(elevatorSubsystem.raiseElevatorIntake()).andThen(elevatorSubsystem.lowerElevator()));
    //Score
    NamedCommands.registerCommand("Score",(new WaitCommand(0.8)).andThen(newIntakeSubsystem.runIntake()).andThen(new WaitCommand(0.2)).andThen(horizontalElevatorSubsystem.MoveHEBack().alongWith(elevatorSubsystem.lowerElevator())));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(m_driverController, Button.kR1.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));

    rTButton = new Trigger(() -> 
      { return m_driverController.getRightTriggerAxis() > 0.5;}
      );

    lTButton = new Trigger(() -> 
      { return m_driverController.getLeftTriggerAxis() > 0.5;}
      );

    if(m_driverController.getLeftTriggerAxis() > 0.5)
      isInCoralMode = !isInCoralMode;

    if (isInCoralMode)
    {
        // intake
        new JoystickButton(m_driverController, XboxController.Button.kX.value)
          .onTrue(newIntakeSubsystem.IntakeCoralSubstation().alongWith(elevatorSubsystem.raiseElevatorIntake())
          .andThen(elevatorSubsystem.lowerElevator()).alongWith(horizontalElevatorSubsystem.MoveHEBack()));
          // L4
        new JoystickButton(m_driverController, XboxController.Button.kY.value).and(new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value))
          .onTrue(
            m_robotDrive.AutoAlignRight().andThen(elevatorSubsystem.raiseElevatorL4().alongWith(horizontalElevatorSubsystem.HElevatorForwardWithL4Clearance())
          ));
        new JoystickButton(m_driverController, XboxController.Button.kY.value).and(new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value))
        .onTrue(
          m_robotDrive.AutoAlignLeft().andThen(elevatorSubsystem.raiseElevatorL4().alongWith(horizontalElevatorSubsystem.HElevatorForwardWithL4Clearance())
        ));
        // L3
        new JoystickButton(m_driverController, XboxController.Button.kB.value).and(new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value))
        .onTrue(
          m_robotDrive.AutoAlignLeft().andThen(elevatorSubsystem.raiseElevatorL3().alongWith(horizontalElevatorSubsystem.HElevatorForwardWithL3Clearance())
        ));
        new JoystickButton(m_driverController, XboxController.Button.kB.value).and(new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value))
        .onTrue(
          m_robotDrive.AutoAlignRight().andThen(elevatorSubsystem.raiseElevatorL3().alongWith(horizontalElevatorSubsystem.HElevatorForwardWithL3Clearance())
        ));
        // L2
        new JoystickButton(m_driverController, XboxController.Button.kA.value).and(new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value))
        .onTrue(
          m_robotDrive.AutoAlignLeft().andThen(elevatorSubsystem.raiseElevatorL2().alongWith(horizontalElevatorSubsystem.HElevatorForwardWithL2Clearance())
        ));
        new JoystickButton(m_driverController, XboxController.Button.kA.value).and(new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value))
        .onTrue(
          m_robotDrive.AutoAlignRight().andThen(elevatorSubsystem.raiseElevatorL2().alongWith(horizontalElevatorSubsystem.HElevatorForwardWithL2Clearance())
        ));
        // new JoystickButton(m_driverController, XboxController.Button.kA.value)
        //   .onTrue(m_robotDrive.AutoAlignLeft());
        //   new JoystickButton(m_driverController, XboxController.Button.kB.value)
        //   .onTrue(m_robotDrive.AutoAlignRight());
        // // testing
     
     
        // Score
      rTButton
        .onTrue(newIntakeSubsystem.runIntake())
        .onFalse(elevatorSubsystem.lowerElevator().alongWith(horizontalElevatorSubsystem.MoveHEBack()));
      

    // new Trigger(isInIntakeZone())
    // .onTrue(newIntakeSubsystem.IntakeCoralSubstation().alongWith(elevatorSubsystem.raiseElevatorIntake())
    // .andThen(elevatorSubsystem.lowerElevator()));
    
    }
    else 
    {
      // new JoystickButton(m_driverController, XboxController.Button.kA.value)
      // .onTrue(m_robotDrive.AutoAlignFar().andThen(m_robotDrive.AutoAlignMiddle()));
      //input algae commands

    }





    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    selectedAuto = autoChooser.getSelected();
    if (selectedAuto == null) {
        selectedAuto = "Move";
    }
    // System.out.println("Selected Auto: " + selectedAuto);
    
    return new PathPlannerAuto(selectedAuto);

  }
}
