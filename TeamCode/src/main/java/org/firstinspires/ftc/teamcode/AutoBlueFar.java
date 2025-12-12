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
        r.intakePower(0.5);
        r.launcherVelocity(950); // should be 1100

        shootBalls(r); // Shoot preloaded balls

        // Pick up balls
        r.driveToPos(-350, 740, 90);
        r.turnLifterToDegrees(0, 600);
        r.intakePower(0.8);
        r.driveToPos(-1300, 740, 90, 15, 3, 3); // slurp the balls up
        r.waitForLifter();
        sleep(700);

        shootBalls(r); // youll never guess what this does

        r.driveToPos(-500, 300, 45); // park
        r.launcherVelocity(0);
        r.intakePower(0);
    }

    private void shootBalls(RockinBot r)
    {
        r.intakePower(0.2);
        r.driveToPos(50, 150, 35); // URGENT: this angle should be 25
        r.intakePower(0.5);
        r.waitForLaunchers(950); // Should be 1100
        r.turnLifterToDegrees(90); // Shoot the first ball
        r.waitForLifter();
        r.waitForLaunchers(950); // Should be 1100
        r.turnLifterToDegrees(180); // Shoot the second ball
        r.waitForLifter();
        r.waitForLaunchers(950); // Should be 1100
        r.turnLifterToDegrees(360); // Shoot the third ball
        r.waitForLifter();
        r.turnLifterToDegrees(-190); // Reset lifter
    }
}