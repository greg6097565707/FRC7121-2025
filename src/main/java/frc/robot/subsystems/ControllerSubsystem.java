package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;

public class ControllerSubsystem extends SubsystemBase{

    XboxController controller;

    boolean isInCoralMode;

    RobotContainer robotContainer;

    public ControllerSubsystem(RobotContainer robotContainer)
    {
        this.robotContainer = robotContainer;
        controller = new XboxController(0);
        isInCoralMode = true;
    }

    public Command rumble()
    {
        return run(
            () -> {
                controller.setRumble(RumbleType.kBothRumble, 0.5);
            }
            );
    }

    public Command stopRumble()
    {
        return run(
            () -> {
                controller.setRumble(RumbleType.kBothRumble, 0);
            }
            );
    }

    public Command switchToCoral()
    {
        // robotContainer.configureButtonBindings();
    return run(
        () -> {this.isInCoralMode = true;}
    );
    }

    public Command switchToAlgae()
    {
        // robotContainer.configureButtonBindings();
    return run(
        () -> {this.isInCoralMode = false;}
    );
    }

    public boolean getMode()
    {
        return isInCoralMode;
    }


    
}
