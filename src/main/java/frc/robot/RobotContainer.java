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
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import frc.robot.subsystems.NewIntakeSubsystem;
import frc.robot.subsystems.RGBSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
  public final DriveSubsystem m_robotDrive = new DriveSubsystem(D_INTAKE_IR);
  public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public static final NewIntakeSubsystem newIntakeSubsystem = new NewIntakeSubsystem();
  private static final RGBSubsystem colorSubsystem = new RGBSubsystem();

  public static ControllerSubsystem controller;
  public static ControllerSubsystem secondaryController;

  

  public boolean isInCoralMode = true;


  // The driver's controller
  public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public static XboxController secondary_driverController = new XboxController(OIConstants.kSecondDriverControllerPort);


  Trigger rTButton;
  Trigger lTButton;

  Trigger hasTag;
  Trigger isCloseEnough;

  Trigger secondRTButton;
  Trigger secondLTButton;

  Trigger coralMode;
  Trigger algaeMode;


  // robot state boolean suppliers
  //old autoalign boolenas
public static BooleanSupplier isAlignedRight() {
    Pose3d targetingYSpeed = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
    return (BooleanSupplier) () -> {
      SmartDashboard.putNumber("y",(targetingYSpeed.getX()));
      SmartDashboard.putNumber("x",(targetingYSpeed.getZ()));
        if (
        ((targetingYSpeed.getX()> 0.02) && (targetingYSpeed.getX() < 0.048)) 
        && (Math.abs(targetingYSpeed.getZ()) < 0.6 )) {
            return true;
        } else return false;
    };
}
public static BooleanSupplier isAlignedLeft() {
  Pose3d targetingYSpeed = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
  return (BooleanSupplier) () -> {
    if (
      ((targetingYSpeed.getX()> -.32) && (targetingYSpeed.getX() < -.27)) 
      && (Math.abs(targetingYSpeed.getZ()) < 0.6 )) {
          return true;
      } else return false;
  };
}
public static BooleanSupplier isAlignedMiddle() {
  Pose3d targetingYSpeed = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
  return (BooleanSupplier) () -> {
    if (
      ((targetingYSpeed.getX()> -.19) && (targetingYSpeed.getX() < -.15)) 
      && (Math.abs(targetingYSpeed.getZ()) < 0.6 )) {
          return true;
      } else return false;
  };
}



// public static BooleanSupplier isNotAlignedLeft() {
//   Pose3d targetingYSpeed = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
//   return (BooleanSupplier) () -> {
//       if (Math.abs(targetingYSpeed.getX()+DriveSubsystem.autoAlignYoffsetLeft) > 0.04
//       &&  (Math.abs(LimelightHelpers.getTY("limelight")+DriveSubsystem.autoAlignXoffset)) > 0.3 ) {
//           return true;
//       } else return false;
//   };
// }
// public static BooleanSupplier isNotAlignedMiddle() {
//   Pose3d targetingYSpeed = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
//   return (BooleanSupplier) () -> {
//       if (Math.abs(targetingYSpeed.getX()) > 0.04
//       &&  (Math.abs(LimelightHelpers.getTY("limelight")+DriveSubsystem.autoAlignXoffset)) > 0.3 ) {
//           return true;
//       } else return false;
//   };
// }
// public static BooleanSupplier isNotAlignedFar() {
//   Pose3d targetingYSpeed = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
//   return (BooleanSupplier) () -> {
//       if (Math.abs(targetingYSpeed.getX()) > 0.04
//       &&  (Math.abs(LimelightHelpers.getTY("limelight")+DriveSubsystem.autoAlignXoffsetFar)) > 0.3 ) {
//           return true;
//       } else return false;
//   };
// }


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
    autoChooser.addOption("NearSide3Coral", "nearSide3Coral");
    autoChooser.addOption("FarSide3Coral", "FarSide3Coral");
    autoChooser.addOption("FarSide3CoralAlgae", "FarSide3CoralAlgae");
    autoChooser.addOption("Move", "Move");
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
            NamedCommands.registerCommand("elevatorCam",elevatorSubsystem.raiseElevatorCamA());
    // //pathplannerL4commands
    NamedCommands.registerCommand("RightL4Score",(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignRight().alongWith(elevatorSubsystem.raiseElevatorL4()))
    .andThen(newIntakeSubsystem.runIntake()).andThen(elevatorSubsystem.lowerElevator())));
    NamedCommands.registerCommand("LeftL4Score", elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignLeft().alongWith(elevatorSubsystem.raiseElevatorL4()))
    .andThen(newIntakeSubsystem.runIntake()).andThen(elevatorSubsystem.lowerElevator())); 
    NamedCommands.registerCommand("LeftL4ScoreAlgae",(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignLeft().alongWith(elevatorSubsystem.raiseElevatorL4()))
    .andThen(newIntakeSubsystem.runIntake()).andThen(m_robotDrive.AutoAlignMiddle().alongWith(elevatorSubsystem.DetermineAlgaePrep()))
    .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.DetermineAlgae()))
    ));
    NamedCommands.registerCommand("HighAlgae",(elevatorSubsystem.raiseElevatorCam().andThen(
      m_robotDrive.AutoAlignMiddle())
      .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.grabHighAlage())
      )));
    NamedCommands.registerCommand("LowAlgae",(elevatorSubsystem.raiseElevatorCam().andThen(
        m_robotDrive.AutoAlignMiddle())
        .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.grabLowAlgae())
        )));
    NamedCommands.registerCommand("elevatorNet",(elevatorSubsystem.raiseElevatorNet()));
    NamedCommands.registerCommand("scoreAlgae",(newIntakeSubsystem.AlgaeOutA().andThen(elevatorSubsystem.raiseElevatorCamA())));
    // //pathplannerL3Commands
    // NamedCommands.registerCommand("RightL3Score", m_robotDrive.AutoAlignRight().andThen(elevatorSubsystem.raiseElevatorL3().alongWith(horizontalElevatorSubsystem.HElevatorForwardWithL3Clearance())));
    // NamedCommands.registerCommand("LeftL3Score", m_robotDrive.AutoAlignLeft().andThen(elevatorSubsystem.raiseElevatorL3().alongWith(horizontalElevatorSubsystem.HElevatorForwardWithL3Clearance())));
    // //pathplnnerL2Commands
    // NamedCommands.registerCommand("RightL2Score", m_robotDrive.AutoAlignRight().andThen(elevatorSubsystem.raiseElevatorL2().alongWith(horizontalElevatorSubsystem.HElevatorForwardWithL2Clearance())));
    // NamedCommands.registerCommand("LeftL2Score", m_robotDrive.AutoAlignLeft().andThen(elevatorSubsystem.raiseElevatorL2().alongWith(horizontalElevatorSubsystem.HElevatorForwardWithL2Clearance())));
    // intake
    NamedCommands.registerCommand("Intake", elevatorSubsystem.lowerElevator().andThen(newIntakeSubsystem.IntakeCoralSubstation()).andThen(elevatorSubsystem.raiseElevatorCam()));
    // //Score
    // NamedCommands.registerCommand("Score",(new WaitCommand(0.8)).andThen(newIntakeSubsystem.runIntake()).andThen(horizontalElevatorSubsystem.MoveHEBack().alongWith(elevatorSubsystem.lowerElevator())));
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
  public void configureButtonBindings() {


    controller = new ControllerSubsystem(m_driverController);
    secondaryController = new ControllerSubsystem(secondary_driverController);

    // new JoystickButton(m_driverController, Button.kR1.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));




    //PRIMARY DRIVE CONTROLLER

    rTButton = new Trigger(() -> 
      { return m_driverController.getRightTriggerAxis() > 0.5;}
      );

    lTButton = new Trigger(() -> 
      { return m_driverController.getLeftTriggerAxis() > 0.5;}
      );

    hasTag = new Trigger(() -> 
    {
      return DriveSubsystem.hasTag().getAsBoolean();
  });

  isCloseEnough = new Trigger(() -> 
  {
    return DriveSubsystem.isCloseEnough().getAsBoolean();
});
    
    

    algaeMode = new Trigger(() ->
    {
      return controller.getMode() == false;
    }
    );



      // new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
      //   .onTrue(controller.switchMode()
      //   .andThen(controller.rumble(.5)).andThen(new WaitCommand(.2).andThen(controller.stopRumble())));

        // new JoystickButton(m_driverController, XboxController.Button.kA.value)
        // .onTrue(newIntakeSubsystem.IntakeCoralSubstation());
        // new JoystickButton(m_driverController, XboxController.Button.kX.value)
        // .onTrue(newIntakeSubsystem.intakeAlgae());
        // new JoystickButton(m_driverController, XboxController.Button.kY.value)
        // .onTrue(newIntakeSubsystem.runIntake());
        // new JoystickButton(m_driverController, XboxController.Button.kB.value)
        // .onTrue(m_robotDrive.AutoAlignMiddle());

        hasTag.and(isCloseEnough)
        .whileTrue(controller.rumble().alongWith(secondaryController.rumble()))
        .onFalse(controller.stopRumble().alongWith(secondaryController.stopRumble()));


        new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value)
        // .toggleOnTrue(m_robotDrive.setDriveSpeed(.5))
        // .toggleOnFalse(m_robotDrive.setDriveSpeed(1.0))
        // .onTrue(controller.longBlip())
        // .onFalse(controller.blip());
        .whileTrue(m_robotDrive.setDriveSpeed(1))
        .whileFalse(m_robotDrive.setDriveSpeed(0.5));


        // left side coral
        new JoystickButton(m_driverController, XboxController.Button.kY.value).and(rTButton.negate()).and(new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).negate()).and(isCloseEnough).and(hasTag)
        .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignLeft().alongWith(elevatorSubsystem.raiseElevatorL4()))
        .andThen(newIntakeSubsystem.runIntake()).andThen(elevatorSubsystem.lowerElevator()));

        new JoystickButton(m_driverController, XboxController.Button.kB.value).and(rTButton.negate()).and(new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).negate()).and(isCloseEnough).and(hasTag)
        .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignLeft().alongWith(elevatorSubsystem.raiseElevatorL3()))
        .andThen(newIntakeSubsystem.runIntake()).andThen(elevatorSubsystem.lowerElevator()));

        new JoystickButton(m_driverController, XboxController.Button.kA.value).and(rTButton.negate()).and(new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).negate()).and(isCloseEnough).and(hasTag)
        .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignLeft().alongWith(elevatorSubsystem.raiseElevatorL2()))
        .andThen(newIntakeSubsystem.runIntake()).andThen(elevatorSubsystem.lowerElevator()));





        // // right side coral
        new JoystickButton(m_driverController, XboxController.Button.kY.value).and(rTButton).and(new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).negate()).and(isCloseEnough).and(hasTag)
        .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignRight().alongWith(elevatorSubsystem.raiseElevatorL4()))
        .andThen(newIntakeSubsystem.runIntake()).andThen(elevatorSubsystem.lowerElevator()));

        new JoystickButton(m_driverController, XboxController.Button.kB.value).and(rTButton).and(new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).negate()).and(isCloseEnough).and(hasTag)
        .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignRight().alongWith(elevatorSubsystem.raiseElevatorL3()))
        .andThen(newIntakeSubsystem.runIntake()).andThen(elevatorSubsystem.lowerElevator()));

        new JoystickButton(m_driverController, XboxController.Button.kA.value).and(rTButton).and(new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).negate()).and(isCloseEnough).and(hasTag)
        .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignRight().alongWith(elevatorSubsystem.raiseElevatorL2()))
        .andThen(newIntakeSubsystem.runIntake()).andThen(elevatorSubsystem.lowerElevator()));

    


        //rightsidecoralalgae
        new JoystickButton(m_driverController, XboxController.Button.kY.value).and(rTButton).and(new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value))
        .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignRight().alongWith(elevatorSubsystem.raiseElevatorL4()))
        .andThen(newIntakeSubsystem.runIntake())
        .andThen(m_robotDrive.AutoAlignMiddle().alongWith(elevatorSubsystem.DetermineAlgaePrep()))
        .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.DetermineAlgae()))
        );

        new JoystickButton(m_driverController, XboxController.Button.kB.value).and(rTButton).and(new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value))
        .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignRight().alongWith(elevatorSubsystem.raiseElevatorL3()))
        .andThen(newIntakeSubsystem.runIntake())
        .andThen(m_robotDrive.AutoAlignMiddle().alongWith(elevatorSubsystem.DetermineAlgaePrep()))
        .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.DetermineAlgae()))
        );

        new JoystickButton(m_driverController, XboxController.Button.kA.value).and(rTButton).and(new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value))
        .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignRight().alongWith(elevatorSubsystem.raiseElevatorL2()))
        .andThen(newIntakeSubsystem.runIntake())
        .andThen(m_robotDrive.AutoAlignMiddle().alongWith(elevatorSubsystem.DetermineAlgaePrep()))
        .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.DetermineAlgae()))
        );




        // //leftsidecoralalgae
        new JoystickButton(m_driverController, XboxController.Button.kY.value).and(rTButton.negate()).and(new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value))
        .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignLeft().alongWith(elevatorSubsystem.raiseElevatorL4()))
        .andThen(newIntakeSubsystem.runIntake())
        .andThen(m_robotDrive.AutoAlignMiddle().alongWith(elevatorSubsystem.DetermineAlgaePrep()))
        .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.DetermineAlgae()))
        );

        new JoystickButton(m_driverController, XboxController.Button.kB.value).and(rTButton.negate()).and(new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value))
        .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignLeft().alongWith(elevatorSubsystem.raiseElevatorL3()))
        .andThen(newIntakeSubsystem.runIntake())
        .andThen(m_robotDrive.AutoAlignMiddle().alongWith(elevatorSubsystem.DetermineAlgaePrep()))
        .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.DetermineAlgae()))
        );

        new JoystickButton(m_driverController, XboxController.Button.kA.value).and(rTButton.negate()).and(new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value))
        .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignLeft().alongWith(elevatorSubsystem.raiseElevatorL2()))
        .andThen(newIntakeSubsystem.runIntake())
        .andThen(m_robotDrive.AutoAlignMiddle().alongWith(elevatorSubsystem.DetermineAlgaePrep()))
        .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.DetermineAlgae()))
        );


        //ALgae
        new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .onTrue(newIntakeSubsystem.alageOut())
        .onFalse(newIntakeSubsystem.Stop().andThen(elevatorSubsystem.lowerElevator()));

        lTButton
        .onTrue(elevatorSubsystem.raiseElevatorNet());

        // new JoystickButton(m_driverController, XboxController.Button.kX.value)
        // .onTrue(newIntakeSubsystem.IntakeCoralSubstation().andThen(elevatorSubsystem.raiseElevatorCam()));

        // lTButton
        // .onTrue(elevatorSubsystem.grabLowAlgae().alongWith(newIntakeSubsystem.intakeAlgae()));

        // new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        // .onTrue(elevatorSubsystem.grabLowAlgae().alongWith(newIntakeSubsystem.intakeAlgae()));

        

        // rTButton
        // .onTrue((elevatorSubsystem.raiseElevatorCam()).andThen(m_robotDrive.AutoAlignRight()));

        // lTButton
        // .onTrue((elevatorSubsystem.raiseElevatorCam()).andThen(m_robotDrive.AutoAlignLeft()));




      // new POVButton(m_driverController, 0)
      //   .onTrue(elevatorSubsystem.raiseElevatorL4()
      //   .andThen(controller.rumble(.5)).andThen(new WaitCommand(.2).andThen(controller.stopRumble()))
      //   );
      
      // new POVButton(m_driverController, 0)
      // .onTrue(elevatorSubsystem.Cancel());
      //   // .until(elevatorSubsystem.finishedMotionMagic()).andThen(controller.rumble(.5)).andThen(new WaitCommand(.2).andThen(controller.stopRumble());
      new POVButton(m_driverController, 0)
        .onTrue(
          elevatorSubsystem.raiseElevatorCam().andThen(
            m_robotDrive.AutoAlignMiddle())
            .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.grabHighAlage())
            )); 
            new POVButton(m_driverController, 90)
            .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(
              m_robotDrive.AutoAlignMiddle())
              .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.grabLowAlgae())
              ));
      new POVButton(m_driverController, 180)
        .onTrue(
          elevatorSubsystem.lowerElevator());
          new POVButton(m_driverController, 270)
          .onTrue(
            elevatorSubsystem.raiseElevatorCam());
      //     // .until(elevatorSubsystem.finishedMotionMagic()).andThen(controller.rumble(.5)).andThen(new WaitCommand(.2).andThen(controller.stopRumble()))
      //   );
      new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(
          elevatorSubsystem.lowerElevator().andThen(newIntakeSubsystem.IntakeCoralSubstation())
          .andThen((secondaryController.blip())
          .alongWith(controller.blip()))
          .andThen(elevatorSubsystem.raiseElevatorCam())
          );
          //elevatorSubsystem.lowerElevator().andThen(newIntakeSubsystem.IntakeCoralSubstation()).andThen(elevatorSubsystem.raiseElevatorCam()).alongWith(secondaryController.blip()));
      //     // .until(elevatorSubsystem.finishedMotionMagic()).andThen(controller.rumble(.5)).andThen(new WaitCommand(.2).andThen(controller.stopRumble()))
      //   );


    //SECONDARY DRIVE CONTROLLER


    secondRTButton = new Trigger(() -> 
        { return secondary_driverController.getRightTriggerAxis() > 0.5;}
        );

    secondLTButton = new Trigger(() -> 
        { return secondary_driverController.getLeftTriggerAxis() > 0.5;}
        );

    
    String secondDriver = "Ryland2";

    if (secondDriver.equals("Ryland")) {

    
      secondLTButton.onTrue(elevatorSubsystem.raiseElevatorCam().andThen(
        m_robotDrive.AutoAlignMiddle())
        .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.grabLowAlgae())
        ));
      secondRTButton.onTrue(
        elevatorSubsystem.raiseElevatorCam().andThen(
        m_robotDrive.AutoAlignMiddle())
        .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.grabHighAlage())
        ));

      new POVButton(secondary_driverController, 180)
      .onTrue(
        elevatorSubsystem.lowerElevator()
      );
      new JoystickButton(secondary_driverController, XboxController.Button.kB.value)
        .onTrue(elevatorSubsystem.Cancel());
      
      new JoystickButton(secondary_driverController, XboxController.Button.kY.value)
        .onTrue(elevatorSubsystem.lowerElevator().andThen(newIntakeSubsystem.IntakeCoralSubstation()).andThen(elevatorSubsystem.raiseElevatorCam()));
      
      new JoystickButton(secondary_driverController, XboxController.Button.kX.value)
        .onTrue(newIntakeSubsystem.runIntake().andThen(elevatorSubsystem.lowerElevator()));

        new JoystickButton(secondary_driverController, XboxController.Button.kA.value)
        .onTrue(elevatorSubsystem.trough());
      
        new JoystickButton(secondary_driverController, XboxController.Button.kLeftBumper.value)
        .onTrue(newIntakeSubsystem.IntakeTrough())
        .onFalse(elevatorSubsystem.lowerElevator().alongWith(newIntakeSubsystem.Stop()));

        new JoystickButton(secondary_driverController, XboxController.Button.kX.value)
        .onTrue(
          elevatorSubsystem.lowerElevator().andThen(newIntakeSubsystem.IntakeCoralSubstation())
          .andThen((secondaryController.blip())
          .alongWith(controller.blip()))
          .andThen(elevatorSubsystem.raiseElevatorCam())
          );

    }

    if (secondDriver.equals("Ryland2")) {

    
    

      new POVButton(secondary_driverController, 180)
      .onTrue(
        elevatorSubsystem.lowerElevator()
      );
      new JoystickButton(secondary_driverController, XboxController.Button.kY.value).and(secondRTButton.negate()).and(new JoystickButton(secondary_driverController, XboxController.Button.kRightBumper.value).negate()).and(isCloseEnough).and(hasTag).and(rTButton).and(secondLTButton)
      .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignLeft().alongWith(elevatorSubsystem.raiseElevatorL4()))
      .andThen(newIntakeSubsystem.runIntake()).andThen(elevatorSubsystem.lowerElevator()));

      new JoystickButton(secondary_driverController, XboxController.Button.kB.value).and(secondRTButton.negate()).and(new JoystickButton(secondary_driverController, XboxController.Button.kRightBumper.value).negate()).and(isCloseEnough).and(hasTag).and(rTButton).and(secondLTButton)
      .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignLeft().alongWith(elevatorSubsystem.raiseElevatorL3()))
      .andThen(newIntakeSubsystem.runIntake()).andThen(elevatorSubsystem.lowerElevator()));

      new JoystickButton(secondary_driverController, XboxController.Button.kA.value).and(secondRTButton.negate()).and(new JoystickButton(secondary_driverController, XboxController.Button.kRightBumper.value).negate()).and(isCloseEnough).and(hasTag).and(rTButton).and(secondLTButton)
      .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignLeft().alongWith(elevatorSubsystem.raiseElevatorL2()))
      .andThen(newIntakeSubsystem.runIntake()).andThen(elevatorSubsystem.lowerElevator()));





      // // right side coral
      new JoystickButton(secondary_driverController, XboxController.Button.kY.value).and(secondRTButton).and(new JoystickButton(secondary_driverController, XboxController.Button.kRightBumper.value).negate()).and(isCloseEnough).and(hasTag).and(rTButton)
      .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignRight().alongWith(elevatorSubsystem.raiseElevatorL4()))
      .andThen(newIntakeSubsystem.runIntake()).andThen(elevatorSubsystem.lowerElevator()));

      new JoystickButton(secondary_driverController, XboxController.Button.kB.value).and(secondRTButton).and(new JoystickButton(secondary_driverController, XboxController.Button.kRightBumper.value).negate()).and(isCloseEnough).and(hasTag).and(rTButton)
      .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignRight().alongWith(elevatorSubsystem.raiseElevatorL3()))
      .andThen(newIntakeSubsystem.runIntake()).andThen(elevatorSubsystem.lowerElevator()));

      new JoystickButton(secondary_driverController, XboxController.Button.kA.value).and(secondRTButton).and(new JoystickButton(secondary_driverController, XboxController.Button.kRightBumper.value).negate()).and(isCloseEnough).and(hasTag).and(rTButton)
      .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignRight().alongWith(elevatorSubsystem.raiseElevatorL2()))
      .andThen(newIntakeSubsystem.runIntake()).andThen(elevatorSubsystem.lowerElevator()));
      new JoystickButton(secondary_driverController, XboxController.Button.kLeftBumper.value)
      .onTrue(elevatorSubsystem.raiseElevatorCamA());
      // alg

      new JoystickButton(secondary_driverController, XboxController.Button.kY.value).and(secondRTButton).and(new JoystickButton(secondary_driverController, XboxController.Button.kLeftStick.value)).and(isCloseEnough).and(hasTag).and(rTButton)
        .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignRight().alongWith(elevatorSubsystem.raiseElevatorL4()))
        .andThen(newIntakeSubsystem.runIntake())
        .andThen(m_robotDrive.AutoAlignMiddle().alongWith(elevatorSubsystem.DetermineAlgaePrep()))
        .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.DetermineAlgae()))
        );

        new JoystickButton(secondary_driverController, XboxController.Button.kB.value).and(secondRTButton).and(new JoystickButton(secondary_driverController, XboxController.Button.kLeftStick.value)).and(isCloseEnough).and(hasTag).and(rTButton)
        .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignRight().alongWith(elevatorSubsystem.raiseElevatorL3()))
        .andThen(newIntakeSubsystem.runIntake())
        .andThen(m_robotDrive.AutoAlignMiddle().alongWith(elevatorSubsystem.DetermineAlgaePrep()))
        .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.DetermineAlgae()))
        );

        new JoystickButton(secondary_driverController, XboxController.Button.kA.value).and(secondRTButton).and(new JoystickButton(secondary_driverController, XboxController.Button.kLeftStick.value)).and(isCloseEnough).and(hasTag).and(rTButton)
        .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignRight().alongWith(elevatorSubsystem.raiseElevatorL2()))
        .andThen(newIntakeSubsystem.runIntake())
        .andThen(m_robotDrive.AutoAlignMiddle().alongWith(elevatorSubsystem.DetermineAlgaePrep()))
        .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.DetermineAlgae()))
        );




        // //leftsidecoralalgae
        new JoystickButton(secondary_driverController, XboxController.Button.kY.value).and(secondRTButton.negate()).and(new JoystickButton(secondary_driverController, XboxController.Button.kLeftStick.value)).and(isCloseEnough).and(hasTag).and(rTButton).and(secondLTButton)
        .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignLeft().alongWith(elevatorSubsystem.raiseElevatorL4()))
        .andThen(newIntakeSubsystem.runIntake())
        .andThen(m_robotDrive.AutoAlignMiddle().alongWith(elevatorSubsystem.DetermineAlgaePrep()))
        .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.DetermineAlgae()))
        );

        new JoystickButton(secondary_driverController, XboxController.Button.kB.value).and(secondRTButton.negate()).and(new JoystickButton(secondary_driverController, XboxController.Button.kLeftStick.value)).and(isCloseEnough).and(hasTag).and(rTButton).and(secondLTButton)
        .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignLeft().alongWith(elevatorSubsystem.raiseElevatorL3()))
        .andThen(newIntakeSubsystem.runIntake())
        .andThen(m_robotDrive.AutoAlignMiddle().alongWith(elevatorSubsystem.DetermineAlgaePrep()))
        .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.DetermineAlgae()))
        );

        new JoystickButton(secondary_driverController, XboxController.Button.kA.value).and(secondRTButton.negate()).and(new JoystickButton(secondary_driverController, XboxController.Button.kLeftStick.value)).and(isCloseEnough).and(hasTag).and(rTButton).and(secondLTButton)
        .onTrue(elevatorSubsystem.raiseElevatorCam().andThen(m_robotDrive.AutoAlignLeft().alongWith(elevatorSubsystem.raiseElevatorL2()))
        .andThen(newIntakeSubsystem.runIntake())
        .andThen(m_robotDrive.AutoAlignMiddle().alongWith(elevatorSubsystem.DetermineAlgaePrep()))
        .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.DetermineAlgae()))
        );
        new JoystickButton(secondary_driverController, XboxController.Button.kX.value)
        .onTrue(
          elevatorSubsystem.lowerElevator().andThen(newIntakeSubsystem.IntakeCoralSubstation())
          .andThen((secondaryController.blip())
          .alongWith(controller.blip()))
          .andThen(elevatorSubsystem.raiseElevatorCam())
          );
      
    }


    if (secondDriver.equals("Ryan")) {

      secondLTButton
        .onTrue(
          elevatorSubsystem.lowerElevator().andThen(newIntakeSubsystem.IntakeCoralSubstation())
          .andThen((secondaryController.blip())
          .alongWith(controller.blip()))
          .andThen(elevatorSubsystem.raiseElevatorCam())
          );

      new JoystickButton(secondary_driverController, XboxController.Button.kA.value).onTrue(elevatorSubsystem.raiseElevatorCam().andThen(
        m_robotDrive.AutoAlignMiddle())
        .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.grabLowAlgae())
        ));
      new JoystickButton(secondary_driverController, XboxController.Button.kY.value).onTrue(
        elevatorSubsystem.raiseElevatorCam().andThen(
        m_robotDrive.AutoAlignMiddle())
        .andThen(newIntakeSubsystem.intakeAlgae().alongWith(elevatorSubsystem.grabHighAlage())
        ));

      secondRTButton
        .onTrue(newIntakeSubsystem.runIntake().andThen(elevatorSubsystem.lowerElevator()));


      
      new JoystickButton(secondary_driverController, XboxController.Button.kLeftBumper.value)
        .onTrue(elevatorSubsystem.Cancel());

      new JoystickButton(secondary_driverController, XboxController.Button.kX.value)
      .onTrue(elevatorSubsystem.raiseElevatorCam());
      

      new POVButton(secondary_driverController, 0)
        .onTrue(
          elevatorSubsystem.trough()
  
        );

      new POVButton(secondary_driverController, 180)
      .onTrue(
        elevatorSubsystem.lowerElevator()
      );

    }

    // new JoystickButton(secondary_driverController, XboxController.Button.kRightBumper.value)
    //   .onTrue(newIntakeSubsystem.runIntake().andThen(elevatorSubsystem.lowerElevator().alongWith(horizontalElevatorSubsystem.MoveHEBack())));

    //     // .onFalse(elevatorSubsystem.lowerElevator().alongWith(horizontalElevatorSubsystem.MoveHEBack()));

    // new JoystickButton(secondary_driverController, XboxController.Button.kLeftBumper.value)
    //     .onTrue(newIntakeSubsystem.intakeAlgae())
    //     .onFalse(elevatorSubsystem.lowerElevator().alongWith(horizontalElevatorSubsystem.MoveHEBack()));


    // new JoystickButton(secondary_driverController, XboxController.Button.kX.value)
    //     .onTrue(newIntakeSubsystem.IntakeCoralSubstation().alongWith(elevatorSubsystem.raiseElevatorIntake())
    //     .andThen(elevatorSubsystem.lowerElevator()).alongWith(horizontalElevatorSubsystem.MoveHEBack()));
    //     // L4
    // new JoystickButton(secondary_driverController, XboxController.Button.kY.value)
    //     .onTrue(
    //       elevatorSubsystem.raiseElevatorL4().alongWith(horizontalElevatorSubsystem.HElevatorForwardWithL4Clearance())
    //     );
    //   // L3
    // new JoystickButton(secondary_driverController, XboxController.Button.kB.value)
    //   .onTrue(
    //     elevatorSubsystem.raiseElevatorL3().alongWith(horizontalElevatorSubsystem.HElevatorForwardWithL3Clearance())
    //   );
    //   // L2
    // new JoystickButton(secondary_driverController, XboxController.Button.kA.value)
    //   .onTrue(
    //    elevatorSubsystem.raiseElevatorL2().alongWith(horizontalElevatorSubsystem.HElevatorForwardWithL2Clearance())
    //   );


      // new POVButton(secondary_driverController, 180)
      // .onTrue(elevatorSubsystem.raiseElevatorLowAlgae().andThen(new WaitCommand(1)).andThen(horizontalElevatorSubsystem.HElevatorForward().alongWith(newIntakeSubsystem.intakeAlgae())).andThen(horizontalElevatorSubsystem.MoveHEBack().alongWith(elevatorSubsystem.lowerElevator())));
      // new POVButton(secondary_driverController, 0)
      // .onTrue(elevatorSubsystem.raiseElevatorHighAlgae().andThen(new WaitCommand(1.5)).andThen(horizontalElevatorSubsystem.HElevatorForward().alongWith(newIntakeSubsystem.intakeAlgae())).andThen(horizontalElevatorSubsystem.MoveHEBack().alongWith(elevatorSubsystem.lowerElevator())));



    




    

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
