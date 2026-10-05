package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name="Remote Control", group="Linear OpMode")
public class RemoteControl extends LinearOpMode {
    @Override

    //Op mode runs when the robot runs. It runs the whole time.
    public void runOpMode() {

        // Create a LinearOpModeVariable and pass it to the RockinBot constructor
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o);
        r.setpValue(50);

        boolean park = false;
        double intakeSpeed = -1;
        //Starting robot shooter speed. Ideal for shooting. Can be adjusted in-game.
        double shooterSpeed = 0.73;
        int shooterDirection = 1;
        double triggerPower = 0;
        boolean previousRightBumper = false;
        boolean previousLeftBumper = false;
        boolean previousTriangle = false;
        boolean previousDpadRight = false;
        boolean previousDpadLeft = false;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Remote Control Ready", "press PLAY");
        RobotLog.vv("Rockin' Robots", "Remote Control Ready");
        telemetry.addData("This code was last updated", "10/4/2026, 5:06 pm"); // Todo: Update this date when the code is updated
        telemetry.update();
        waitForStart();
        r.intakePower(intakeSpeed);
        r.shooterPower(shooterSpeed * shooterDirection);

        // Timer used to throttle telemetry updates to every half-second
        ElapsedTime telemetryTimer = new ElapsedTime();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // r.start();
            boolean speedChanged = false;

            if(gamepad1.dpad_down) {
                park = true;
            } else if(gamepad1.dpad_up) {
                park = false;
            }
            r.setWheelPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if(gamepad1.right_trigger > 0)
            {
                triggerPower = 1;
                r.triggerPower(triggerPower);
            }

            if(gamepad1.left_trigger > 0)
            {
                triggerPower = -1;
                r.triggerPower(triggerPower);
            }

            if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0){
                triggerPower = 0;
                r.triggerPower(triggerPower);
            }

            if(gamepad1.circle){
                intakeSpeed = 1;
                r.intakePower(intakeSpeed);
            }
            else if(gamepad1.square){
                intakeSpeed = -1;
                r.intakePower(intakeSpeed);
            }
            else if(gamepad1.cross){
                intakeSpeed = 0;
                r.intakePower(intakeSpeed);
            }

            if (gamepad1.right_bumper && !previousRightBumper) {
                shooterSpeed = Math.min(1.0, shooterSpeed + 0.01);
                speedChanged = true;
            }
            if (gamepad1.left_bumper && !previousLeftBumper) {
                shooterSpeed = Math.max(0.0, shooterSpeed - 0.01);
                speedChanged = true;
            }
            if (gamepad1.triangle && !previousTriangle) {
                shooterDirection *= -1;
                speedChanged = true;
            }
            if (gamepad1.right_stick_button) {
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
            previousTriangle = gamepad1.triangle;
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