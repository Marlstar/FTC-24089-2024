package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TestOpMode1 extends LinearOpMode
{
    // MOVEMENT
    private DcMotor frontLeftDriveMotor;
    private DcMotor frontRightDriveMotor;
    private DcMotor backLeftDriveMotor;
    private DcMotor backRightDriveMotor;

    // INTAKE
    // Spin motors (1150RPMs)
    private DcMotor leftIntakeMotor;
    private DcMotor rightIntakeMotor;
    // 



    @Override
    public void runOpMode() throws InterruptedException
    {
        // Initialising motors
        // Drive motors
        frontLeftDriveMotor = hardwareMap.get(DcMotor.class, "frontLeftDriveMotor");
        frontRightDriveMotor = hardwareMap.get(DcMotor.class, "frontRightDriveMotor");
        backLeftDriveMotor = hardwareMap.get(DcMotor.class, "backLeftDriveMotor");
        backRightDriveMotor = hardwareMap.get(DcMotor.class, "backRightDriveMotor");
        // Intake motors
        leftIntakeMotor = hardwareMap.get(DcMotor.class, "leftIntakeMotor");
        rightIntakeMotor = hardwareMap.get(DcMotor.class, "rightIntakeMotor");


        waitForStart();

        // Main loop
        while (opModeIsActive())
        {

        }
    }
}
