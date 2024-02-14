package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Custom
import org.firstinspires.ftc.teamcode.Core.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.Robot;

@TeleOp
@Disabled
public class BaseOpMode extends LinearOpMode
{
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.create(hardwareMap);

        // Runs when "init" is pressed
        robot.drive.motors.init();
        robot.drive.robotIMU.init();

        if (isStopRequested()) return;
        waitForStart();
        //////////////////////////////////////////
        // Runs when the play button is pressed //
        //////////////////////////////////////////

        // Main loop
        while (opModeIsActive())
        {
            robot.controllerInput.mainloop(gamepad1);
            robot.drive.movement.moveTele();
        }
    }
}
