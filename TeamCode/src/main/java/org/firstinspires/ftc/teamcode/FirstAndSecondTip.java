package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="First and Second Tip Auto", group="Robot")
public class FirstAndSecondTip extends LinearOpMode {
    private static final double LEFT_SHOOTER_VELOCITY = 1725;
    private static final double RIGHT_SHOOTER_VELOCITY = 1725;
    private static final double TRIGGER_POWER = 1;
    private static final double INTAKE_POWER = 0.8;
    //Note: the lumberajcjk position stuff is commented out for now pls change to set left and right
    @Override
    public void runOpMode() {
        RockinBot robot = new RockinBot(this);

        telemetry.addData("Autonomous Ready", "Shoot, then park");
        telemetry.addData("Version", "v20261110-02");
        telemetry.update();
        waitForStart();

        // Start motors
        robot.shooterVelocity(LEFT_SHOOTER_VELOCITY, RIGHT_SHOOTER_VELOCITY);
        robot.setLeftPValue(70);
        robot.setRightPValue(100);
        sleep(4000); // let shooters get up to speed

        // Shoot 1st round: balls 1, 2, 3, & 4
        robot.intakePower(INTAKE_POWER);
        robot.triggerPower(TRIGGER_POWER);
        sleep(1200);
        robot.triggerPower(0);

        // Collect balls from corner
        robot.driveToPos(0, -150, 0, 25, 5, 1, 0.5); // Waypoint to get off the wall
        robot.driveToPos(400, -20, -85, 50,2, 3, 0.7); // Move to collect balls
        robot.driveToPos(1100, 0, -85, 50, 2, 2, 0.5); // Collect them
        sleep(1000);

        // Shoot 3rd round: balls 5, 6, 7, & 8
        robot.driveToPos(330, -2500, 180, 100, 5, 3, 1); // Waypoint to avoid the hive
        robot.driveToPos(-550, -2500, 180, 50, 3, 2, 0.7); // Waypoint to avoid the flower
        robot.driveToPos(-550, -3000, 180, 20, 1, 3, 0.5); // Go to new shooting position
        sleep(100);
        robot.triggerPower(TRIGGER_POWER);
        sleep(1000);
        robot.triggerPower(0);
        sleep(500);

        /*
        // Park
        RobotLog.vv("Rockin' Robots", "Drive to the parking box");
        robot.driveToPos(1150, -2150, 0, 50, 2, 4, 0.7);
        robot.intakePower(0);
*/
        robot.odo.update();
        Pose2D pose = robot.odo.getPosition();
        telemetry.addData("X (mm)", "%.1f", pose.getX(DistanceUnit.MM));
        telemetry.addData("Y (mm)", "%.1f", pose.getY(DistanceUnit.MM));
        telemetry.addData("Heading (degrees)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }
}