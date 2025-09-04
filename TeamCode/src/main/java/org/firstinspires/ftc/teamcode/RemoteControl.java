package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Remote Control", group="Linear OpMode")
public class RemoteControl extends LinearOpMode {
    @Override

    //Op mode runs when the robot runs. It runs the whole time.
    public void runOpMode() {
        // Create a LinearOpModeVariable and pass it to the RockinBot constructor
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Remote Control Ready", "press PLAY");
        telemetry.addData("This code was last updated", "3/16/2024, 4:30 pm"); // Todo: Update this date when the code is updated
        telemetry.update();
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            r.setWheelPower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // Show the elapsed game time and wheel power.
            r.printDataOnScreen();
        }
    }
}