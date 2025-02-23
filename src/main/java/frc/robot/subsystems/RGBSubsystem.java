package frc.robot.subsystems;

import org.w3c.dom.css.RGBColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.IntakeIR;
import frc.robot.RobotContainer;
import frc.robot.controllers.RGBController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;





public class RGBSubsystem extends SubsystemBase {
    private final RGBController m_blinkinDriver = new RGBController();
    public RGBSubsystem() {
        this.idle();
    }
    public void idle() {
        m_blinkinDriver.set(.15);
    }
    public void GamePieceIN() {
        //red
        m_blinkinDriver.set(.77);
    }
    public void ScoringCoral() {
        //yellow
        m_blinkinDriver.set(.15);
    }
    public void GrabbingAlgae() {
        //green
        m_blinkinDriver.set(.35);
    }
    public void GamePieceOut() {
        m_blinkinDriver.set(.61);
    
    }
     //commands
    public Command GamePieceINCommand(){
        return run(this::GamePieceIN);
    }
    public Command GamePieceOutCommand(){
        return run(this::GamePieceOut);
    }
    public Command ScoringCoralCommand(){
        return run(this::ScoringCoral);
    }
    public Command ScoringAlgaeCommand(){
        return run(this::GrabbingAlgae);
    }
    public void periodic() {
       
    }
}