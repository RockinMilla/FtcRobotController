package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name="Remote Control Shooter", group="Linear OpMode")
public class RemoteControlShooter extends LinearOpMode {
    @Override

    //Op mode runs when the robot runs. It runs the whole time.
    public void runOpMode() {

        // Create a LinearOpModeVariable and pass it to the RockinBot constructor
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o, "Shooter");

        // THESE ARE THE VARIABLES THAT ARE RUNNING DURING RC, NOT THE ONES IN ROCKINBOT!!
        // These are the defaults that run when the program starts. Their values can be modified by RC inputs\
        boolean park = false;
        double intakeSpeed = 1;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Remote Control Ready", "press PLAY");
        RobotLog.vv("Rockin' Robots", "Remote Control Ready");
        telemetry.addData("This code was last updated", "1/8/2026, 4:35 pm"); // Todo: Update this date when the code is updated
        telemetry.update();
        waitForStart();
        r.intakePower(intakeSpeed);

        // Timer used to throttle telemetry updates to every half-second
        ElapsedTime telemetryTimer = new ElapsedTime();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            r.start();

            if(gamepad1.dpad_down) {
                park = true;
            } else if(gamepad1.dpad_up) {
                park = false;
            }
            r.setWheelPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, park);

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

            /////////////////////////////////////////////////////////////

            /*if(gamepad2.dpad_down) {
                r.adjustpValue(-0.1);
            } else if(gamepad2.dpad_up) {
                r.adjustpValue(0.1);
            } else if(gamepad2.dpad_right) {
                launcherSpeed += 10;
                r.launcherVelocity(launcherSpeed);
            } else if(gamepad2.dpad_left) {
                launcherSpeed -= 10;
                r.launcherVelocity(launcherSpeed);
            }*/

            // Show the elapsed game time and wheel power, but only every half-second.
            if (telemetryTimer.seconds() >= 0.5) {
                r.printDataOnScreen();
                telemetryTimer.reset();
            }
        }
    }
}