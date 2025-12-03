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
        RockinBot r = new RockinBot(o, "Shooter");

        // THESE ARE THE VARIABLES THAT ARE RUNNING DURING RC, NOT THE ONES IN ROCKINBOT!!
        // These are the defaults that run when the program starts. Their values can be modified by RC inputs
        double launcherSpeed = 500; // figure this out
        double intakeSpeed = 1;
        double lifterPower = 0;
        boolean park = false;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Remote Control Ready", "press PLAY");
        RobotLog.vv("Rockin' Robots", "Remote Control Ready");
        telemetry.addData("This code was last updated", "11/3/2025, 10:10 am"); // Todo: Update this date when the code is updated
        telemetry.update();
        waitForStart();
        r.intakePower(intakeSpeed);
        r.launcherVelocity(launcherSpeed);
        r.lifterPower(lifterPower);

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.dpad_right) {
                r.turnLifterByDegrees(90);
                sleep(2000);
            }
            else if (gamepad1.dpad_left) {
                r.turnLifterByDegrees(360);
                sleep(2000);
            }
            else if (gamepad1.dpad_down){
                r.turnLifterByDegrees(180);
                sleep(2000);
            }
            else if (gamepad1.dpad_up){
                r.turnLifterByDegrees(45);
                sleep(2000);
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

            if(gamepad1.right_trigger > 0){
                lifterPower = 0.35;
                r.lifterPower(lifterPower);
            }

            if (gamepad1.left_trigger > 0){
                lifterPower = -0.35;
                r.lifterPower(lifterPower);
            }

            if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0){
                lifterPower = 0;
                r.lifterPower(lifterPower);
            }

            if(gamepad1.dpad_down){
                park = true;
            }
            else if(gamepad1.dpad_up){
                park = false;
            }
            r.setWheelPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, park);

            // Show the elapsed game time and wheel power.
            r.printDataOnScreen();
        }
    }
}