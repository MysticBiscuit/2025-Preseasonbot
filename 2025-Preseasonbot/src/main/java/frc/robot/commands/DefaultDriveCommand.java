// Just Copied from last year but we can edit it
// Might want to change things like variable names but most of it should be good idk.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultDriveCommand extends Command {
    private final DriveSubsystem m_drive;
    private final XboxController m_controller;
    private final IntakeSubsystem m_intake;

    public DefaultDriveCommand(DriveSubsystem driveSubsystem, IntakeSubsystem intake, XboxController controller) {
        m_drive = driveSubsystem;
        m_controller = controller;
        m_intake = intake;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        // Get alliance color
        boolean isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        
        // Get joystick inputs with deadband
        double xSpeed = -MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.kDriveDeadband);
        double ySpeed = -MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.kDriveDeadband); 
        double rot = -MathUtil.applyDeadband(m_controller.getRightX(), OIConstants.kDriveDeadband);

        // Invert x and y inputs if on red alliance
        if (isRed) {
            xSpeed = -xSpeed;
            ySpeed = -ySpeed;
        }

        m_intake.setStickState(m_controller.getPOV());

        if (m_controller.getRawButton(7)) {
            System.out.println("RESETTING GYRO");
            m_drive.m_poseEstimator.m_gyro.setYaw(0);
        }

        m_drive.drive(xSpeed, ySpeed, rot, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
