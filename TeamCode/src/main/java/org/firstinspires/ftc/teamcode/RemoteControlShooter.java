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

        // We don't know why, but EDIT THESE!!! THESE ARE THE VARIABLES THAT ARE RUNNING DURING RC, NOT THE ONES IN ROCKINBOT!!
        // The ones in RockinBot are supposed to be running, but these are instead, so don't question it! If it works, it works!
        // These are the defaults that run when the program starts. Their values can be modified by RC inputs
        double launcherSpeed = 0.5;
        double intakeSpeed = -0.6;
        double lifterPower = 0.05;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Remote Control Ready", "press PLAY");
        RobotLog.vv("Rockin' Robots", "Remote Control Ready");
        telemetry.addData("This code was last updated", "11/1/2025, 1:41 pm"); // Todo: Update this date when the code is updated
        telemetry.update();
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.dpad_right) {
                launcherSpeed += 0.001;
            }
            else if (gamepad1.dpad_left) {
                launcherSpeed -= 0.001;
            }

            if(gamepad1.circle){
                intakeSpeed = -0.6;
            }
            else if(gamepad1.square){
                intakeSpeed = 0;
            }

            if(gamepad1.right_trigger > 0){
                lifterPower = -1;
            }

            if (gamepad1.left_trigger > 0){
                lifterPower = 1;
            }

            if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0){
                lifterPower = 0.05;
            }
            
            r.setWheelPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            r.launcherPower(launcherSpeed);
            r.intakePower(intakeSpeed);
            r.lifterPower(lifterPower);

            // Show the elapsed game time and wheel power.
            r.printDataOnScreen();

            RobotLog.vv("Rockin' Robots", "Wheel Power: %.2f, %.2f, %.2f, %.2f");
        }
    }
}