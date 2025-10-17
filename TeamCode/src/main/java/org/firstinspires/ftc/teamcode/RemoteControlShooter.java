package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name="Remote Control Shooter", group="Linear OpMode")
public class RemoteControlShooter extends LinearOpMode {
    @Override

    //Op mode runs when the robot runs. It runs the whole time.
    public void runOpMode() {
        // Create a LinearOpModeVariable and pass it to the RockinBot constructor
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o);

        double launcherSpeed = 1;
        double intakeSpeed = -0.57;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Remote Control Ready", "press PLAY");
        RobotLog.vv("Rockin' Robots", "Remote Control Ready");
        telemetry.addData("This code was last updated", "9/18/2025, 5:30 pm"); // Todo: Update this date when the code is updated
        telemetry.update();
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                launcherSpeed += 0.001;
            }
            else if (gamepad1.left_bumper) {
                launcherSpeed -= 0.001;
            }

            if(gamepad1.right_trigger > 0){
                intakeSpeed += 0.01;
            }
            else if(gamepad1.left_trigger  > 0){
                intakeSpeed -= 0.01;
            }
            r.launcherPower(launcherSpeed);
            r.intakePower(intakeSpeed);
            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Launcher Power", "%.2f", launcherSpeed);
            telemetry.addData("Intake Power", "%.2f", intakeSpeed);
            telemetry.update();
            r.printDataOnScreen();
            RobotLog.vv("Rockin' Robots", "Wheel Power: %.2f, %.2f, %.2f, %.2f");
        }
    }
}