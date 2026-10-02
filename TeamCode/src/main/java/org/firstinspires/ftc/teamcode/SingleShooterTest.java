package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name="Single Shooter Test", group="Linear OpMode")
public class SingleShooterTest extends LinearOpMode {
    @Override

    //Op mode runs when the robot runs. It runs the whole time.
    public void runOpMode() {

        // Create a LinearOpModeVariable and pass it to the RockinBot constructor
        LinearOpMode o = this;
        RockinBotTest r = new RockinBotTest(o);

        double shooterSpeed = 0.66;
        int shooterDirection = 1;
        boolean previousRightBumper = false;
        boolean previousLeftBumper = false;
        boolean previousSquare = false;
        boolean previousDpadRight = false;
        boolean previousDpadLeft = false;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Remote Control Ready", "press PLAY");
        RobotLog.vv("Rockin' Robots", "Remote Control Ready");
        telemetry.addData("This code was last updated", "9/23/2026"); // Todo: Update this date when the code is updated
        telemetry.update();
        waitForStart();
        r.shooterPower(shooterSpeed * shooterDirection);

        // Timer used to throttle telemetry updates to every half-second
        ElapsedTime telemetryTimer = new ElapsedTime();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            boolean speedChanged = false;

            if (gamepad1.right_bumper && !previousRightBumper) {
                shooterSpeed = Math.min(1.0, shooterSpeed + 0.05);
                speedChanged = true;
            }
            if (gamepad1.left_bumper && !previousLeftBumper) {
                shooterSpeed = Math.max(0.0, shooterSpeed - 0.05);
                speedChanged = true;
            }
            if (gamepad1.square && !previousSquare) {
                shooterDirection *= -1;
                speedChanged = true;
            }
            if (gamepad1.cross) {
                shooterSpeed = 0.0;
                speedChanged = true;
            }

            if (gamepad1.dpad_right && !previousDpadRight) {
                r.adjustpValue(2);
            }
            if (gamepad1.dpad_left && !previousDpadLeft) {
                r.adjustpValue(-2);
            }

            if (speedChanged) {
                r.shooterPower(shooterSpeed * shooterDirection);
            }

            previousRightBumper = gamepad1.right_bumper;
            previousLeftBumper = gamepad1.left_bumper;
            previousSquare = gamepad1.square;
            previousDpadRight = gamepad1.dpad_right;
            previousDpadLeft = gamepad1.dpad_left;

            // Show the elapsed game time and wheel power, but only every half-second.
            if (telemetryTimer.seconds() >= 0.5) {
                r.printDataOnScreen();
                telemetryTimer.reset();
            }
        }
    }
}