package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;

public class ControllerSubsystem extends SubsystemBase{

    XboxController controller;

    boolean isInCoralMode;

    RobotContainer robotContainer;

    public ControllerSubsystem(XboxController controller)
    {
        // this.robotContainer = robotContainer;
        this.controller = controller;
        isInCoralMode = true;
    }

    public Command rumble()
    {
        return runOnce(
            () -> {
                controller.setRumble(RumbleType.kBothRumble, 1);
            }
            );
    }

    public Command rumble(double strength)
    {
        return runOnce(
            () -> {
                controller.setRumble(RumbleType.kBothRumble, strength);
            }
            );
    }

    public Command stopRumble()
    {
        return runOnce(
            () -> {
                controller.setRumble(RumbleType.kBothRumble, 0);
            }
            );
    }

    public Command blip()
    {
        return runOnce(
            () -> {
                controller.setRumble(RumbleType.kBothRumble, 1);
            }
            ).andThen(new WaitCommand(.5).andThen(stopRumble()));
    }

    public Command longBlip()
    {
        return runOnce(
            () -> {
                controller.setRumble(RumbleType.kBothRumble, 1);
            }
            ).andThen(new WaitCommand(1).andThen(stopRumble()));
    }

    public Command tinyBlip()
    {
        return runOnce(
            () -> {
                controller.setRumble(RumbleType.kBothRumble, 1);
            }
            ).andThen(new WaitCommand(.25).andThen(stopRumble()));
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

    public Command switchMode()
    {
        // robotContainer.configureButtonBindings();
    return runOnce(
        () -> {this.isInCoralMode = !isInCoralMode;
    }
    );
    }

    public boolean getMode()
    {
        return isInCoralMode;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Coral Mode", String.valueOf(isInCoralMode));


        
        // else {
        //     stopRumble();
        // }

    }


    
}
