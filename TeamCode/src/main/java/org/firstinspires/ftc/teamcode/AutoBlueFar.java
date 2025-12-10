package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Far", group="Robot")
public class AutoBlueFar extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        LinearOpMode o = this;
        RockinBot r = new RockinBot(o, "Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "12/3/2025, 2:20 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        r.getPinpointPosition();
        //r.lifterPower(-0.1);
        r.intakePower(0.5);
        r.launcherVelocity(1100); // finetune this

        shootBalls(r); // Shoot preloaded balls

        // Pick up 1st set of balls
        r.driveToPos(-350, 730, 90);
        r.turnLifterByDegrees(270, 500);
        sleep(100);
        r.driveToPos(-1300, 730, 90, 15, 3, 3); // slurp the balls up
        sleep(300);
        r.turnLifterByDegrees(-50);

        shootBalls(r); // youll never guess what this does

        r.driveToPos(-400, 300, 25); // park
        r.launcherVelocity(0);
        r.intakePower(0);
    }

    private void shootBalls(RockinBot r)
    {
        r.driveToPos(0, 100, 30); // URGENT: this angle should be 25
        r.launcherVelocity(900); // URGENT: this velocity should be 1300
        sleep(1000);
        r.turnLifterByDegrees(90, 2000);
        sleep(700);
        r.turnLifterByDegrees(90, 2000);
        sleep(700);
        r.turnLifterByDegrees(360, 2000);
        sleep(2000);
    }
}