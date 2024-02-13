package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    public ControllerInput controllerInput;
    HardwareMap hardwareMap;

    /**
     * @param h A reference to the "hardwareMap"
     */
    public void create(HardwareMap h)
    {
        hardwareMap = h;
    }

    public class Drive {
        public class Motors {
            // MOVEMENT
            // Drive motors
            public DcMotor frontLeftDriveMotor;
            public DcMotor frontRightDriveMotor;
            public DcMotor backLeftDriveMotor;
            public DcMotor backRightDriveMotor;

            // INTAKE
            // Spin motors (1150RPMs)
            public DcMotor leftIntakeSpinMotor;
            public DcMotor rightIntakeSpinMotor;
            // Position motors
            public Servo leftIntakeRaise;
            public Servo rightIntakeRaise;

            public void init() {
                // Initialising motors
                // MOVEMENT
                // Drive motors
                frontLeftDriveMotor = hardwareMap.get(DcMotor.class, "frontLeftDriveMotor");
                frontRightDriveMotor = hardwareMap.get(DcMotor.class, "frontRightDriveMotor");
                backLeftDriveMotor = hardwareMap.get(DcMotor.class, "backLeftDriveMotor");
                backRightDriveMotor = hardwareMap.get(DcMotor.class, "backRightDriveMotor");
                // INTAKE
                // Spin motors (1150RPMs)
                leftIntakeSpinMotor = hardwareMap.get(DcMotor.class, "leftIntakeMotor");
                rightIntakeSpinMotor = hardwareMap.get(DcMotor.class, "rightIntakeMotor");
                // Raise servos
                leftIntakeRaise = hardwareMap.get(Servo.class, "leftIntakeRaise");
                rightIntakeRaise = hardwareMap.get(Servo.class, "rightIntakeRaise");
            }
        } public Motors motors;

        public class Movement {
            class DesiredMovements {
                public double frontLeftDrive;
                public double frontRightDrive;
                public double backLeftDrive;
                public double backRightDrive;
            } public DesiredMovements desiredMovements;

            void calculateMovementTele() {
                double mx = controllerInput.movement_x;
                double my = controllerInput.movement_y;
                double r = controllerInput.rotation;

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(my) + Math.abs(mx) + Math.abs(r), 1);

                desiredMovements.frontLeftDrive = (mx + my + r) / denominator;
                desiredMovements.frontRightDrive = (mx - my - r) / denominator;
                desiredMovements.backLeftDrive = (mx - my + r) / denominator;
                desiredMovements.backRightDrive = (mx + my - r) / denominator;
            }

            public void setMotorPowers()
            {
                motors.frontLeftDriveMotor.setPower(desiredMovements.frontLeftDrive);
                motors.frontRightDriveMotor.setPower(desiredMovements.frontRightDrive);
                motors.backLeftDriveMotor.setPower(desiredMovements.backLeftDrive);
                motors.backRightDriveMotor.setPower(desiredMovements.backRightDrive);
            }

            public void moveTele()
            {
                calculateMovementTele();
                setMotorPowers();
            }
        } public Movement movement;
    } public Drive drive;
}
