package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name="Single Motor Test", group="Linear OpMode")
public class SingleMotorTest extends LinearOpMode {
    @Override

    //Op mode runs when the robot runs. It runs the whole time.
    public void runOpMode() {

        // Create a LinearOpModeVariable and pass it to the RockinBot constructor
        LinearOpMode o = this;
        RockinBotTest r = new RockinBotTest(o);

        double motorSpeed = 1.0;
        int motorDirection = 1;
        boolean previousRightBumper = false;
        boolean previousLeftBumper = false;
        boolean previousSquare = false;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Remote Control Ready", "press PLAY");
        RobotLog.vv("Rockin' Robots", "Remote Control Ready");
        telemetry.addData("This code was last updated", "9/23/2026"); // Todo: Update this date when the code is updated
        telemetry.update();
        waitForStart();
        r.motorPower(motorSpeed * motorDirection);

        // Timer used to throttle telemetry updates to every half-second
        ElapsedTime telemetryTimer = new ElapsedTime();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            boolean speedChanged = false;

            if (gamepad1.right_bumper && !previousRightBumper) {
                motorSpeed = Math.min(1.0, motorSpeed + 0.05);
                speedChanged = true;
            }
            if (gamepad1.left_bumper && !previousLeftBumper) {
                motorSpeed = Math.max(0.0, motorSpeed - 0.05);
                speedChanged = true;
            }
            if (gamepad1.square && !previousSquare) {
                motorDirection *= -1;
                speedChanged = true;
            }
            if (gamepad1.cross) {
                motorSpeed = 0.0;
                speedChanged = true;
            }

            if (speedChanged) {
                r.motorPower(motorSpeed * motorDirection);
            }

            previousRightBumper = gamepad1.right_bumper;
            previousLeftBumper = gamepad1.left_bumper;
            previousSquare = gamepad1.square;

            // Show the elapsed game time and wheel power, but only every half-second.
            if (telemetryTimer.seconds() >= 0.5) {
                r.printDataOnScreen();
                telemetryTimer.reset();
            }
        }
    }
}