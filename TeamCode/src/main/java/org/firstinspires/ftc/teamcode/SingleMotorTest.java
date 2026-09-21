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

        // THESE ARE THE VARIABLES THAT ARE RUNNING DURING RC, NOT THE ONES IN ROCKINBOT!!
        // These are the defaults that run when the program starts. Their values can be modified by RC inputs\
        boolean park = false;
        double motorSpeed = 1;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Remote Control Ready", "press PLAY");
        RobotLog.vv("Rockin' Robots", "Remote Control Ready");
        telemetry.addData("This code was last updated", "9/20/2026, 4:35 pm"); // Todo: Update this date when the code is updated
        telemetry.update();
        waitForStart();
        r.motorPower(motorSpeed);

        // Timer used to throttle telemetry updates to every half-second
        ElapsedTime telemetryTimer = new ElapsedTime();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.circle){
                motorSpeed = 1;
                r.motorPower(motorSpeed);
            }
            else if(gamepad1.square){
                motorSpeed = -1;
                r.motorPower(motorSpeed);
            }
            else if(gamepad1.cross){
                motorSpeed = 0;
                r.motorPower(motorSpeed);
            }

            // Show the elapsed game time and wheel power, but only every half-second.
            if (telemetryTimer.seconds() >= 0.5) {
                r.printDataOnScreen();
                telemetryTimer.reset();
            }
        }
    }
}