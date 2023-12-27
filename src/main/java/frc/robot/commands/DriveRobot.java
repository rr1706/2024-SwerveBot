package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.MathUtils;

public class DriveRobot extends Command {
    private final Drivetrain m_drivetrain;
    private final CommandXboxController m_controller;

    public DriveRobot(Drivetrain drivetrain, CommandXboxController controller) {
        m_drivetrain = drivetrain;
        m_controller = controller;
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        double desiredTrans[] = MathUtils.inputTransform(-m_controller.getLeftY(), -m_controller.getLeftX());
        double maxLinear = DriveConstants.kMaxSpeedMetersPerSecond;

        desiredTrans[0] *= maxLinear;
        desiredTrans[1] *= maxLinear;

        double desiredRot = -MathUtils.inputTransform(m_controller.getRightX()) * DriveConstants.kMaxAngularSpeed;

        m_drivetrain.drive(desiredTrans[0], desiredTrans[1], desiredRot, true, true);

        SmartDashboard.putBoolean("DrivingByController", true);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("DrivingByController", false);
    }
}
