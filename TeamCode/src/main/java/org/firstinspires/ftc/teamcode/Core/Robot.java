package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// All the imports the IMU wanted:
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class Robot {
    public ControllerInput controllerInput;
    public HardwareMap hwMap;

    /**
     * @param h A reference to the "hardwareMap"
     */
    public void create(HardwareMap h)
    {
        hwMap = h;
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
                frontLeftDriveMotor = hwMap.get(DcMotor.class, "frontLeftDriveMotor");
                frontRightDriveMotor = hwMap.get(DcMotor.class, "frontRightDriveMotor");
                backLeftDriveMotor = hwMap.get(DcMotor.class, "backLeftDriveMotor");
                backRightDriveMotor = hwMap.get(DcMotor.class, "backRightDriveMotor");
                // INTAKE
                // Spin motors (1150RPMs)
                leftIntakeSpinMotor = hwMap.get(DcMotor.class, "leftIntakeMotor");
                rightIntakeSpinMotor = hwMap.get(DcMotor.class, "rightIntakeMotor");
                // Raise servos
                leftIntakeRaise = hwMap.get(Servo.class, "leftIntakeRaise");
                rightIntakeRaise = hwMap.get(Servo.class, "rightIntakeRaise");
            }
        } public Motors motors;

        public class IMU {
            public BNO055IMU imu; //define IMU (for GYRO).
            public double targetYaw;
            // YAW position
            // This will be modified by the user with joysticks, and the robot will use a
            // PID controller to align itself with the target YAW.

            public void resetIMU(){
                // Just a wrapper function to look nice
                initIMU();
            }

            public void initIMU() {
                // Reset and initalize the IMU.
                // It wanted parameters so I create a new parameter obj
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                parameters.calibrationDataFile = "BNO055IMUCalibration.json";
                parameters.loggingEnabled = true;
                parameters.loggingTag = "IMU";
                parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
                imu = hwMap.get(BNO055IMU.class, "imu");
                // This resets it
                imu.initialize(parameters);
            }
            // This is copied from my block code last year.
            public double getYawDegrees() {
                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                return AngleUnit.DEGREES.normalize(angles.firstAngle);
            }

            // Another wrapper function
            public void init() {
                initIMU();
            }

        } public IMU robotIMU;


        public class Movement {
            class DesiredMovements {
                public double frontLeftDrive;
                public double frontRightDrive;
                public double backLeftDrive;
                public double backRightDrive;
            } public DesiredMovements desiredMovements;

            public double calculateRotationPower(double targetRotation, double threshold) {
                double currentRotation = robotIMU.getYawDegrees();
                double deltaRotation = targetRotation - currentRotation;
                // Such that if the robot is at 15d and the targetRotation = 0d,
                // this will be equal to -15d. For now I'll use a > or < but once
                // it works, I will switch to PID.

                if (deltaRotation < -threshold) { return -0.2; }
                if (deltaRotation >  threshold) { return  0.2; }
                else { return 0.0; }
            }

            void calculateMovementTele() {
                double mx = controllerInput.movement_x;
                double my = controllerInput.movement_y;
                //double r = controllerInput.rotation; <- Leave commented out for now
                double r = calculateRotationPower(0.0, 3.0);

                // For now disregard R and focus on making YAW = 0

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(my) + Math.abs(mx) + Math.abs(r), 1);

                //If we are correcting yaw, slow down the motors so that it makes a difference
                // (If they are all 100% speed then decrease to 80% so that the motors can speed up
                // more to rotate). Learnt this last year.
                double movementMultiplier = 1.0 - Math.abs(r);

                desiredMovements.frontLeftDrive = ((mx + my) * movementMultiplier + r) / denominator;
                desiredMovements.frontRightDrive = ((mx - my) * movementMultiplier - r) / denominator;
                desiredMovements.backLeftDrive = ((mx - my) * movementMultiplier + r) / denominator;
                desiredMovements.backRightDrive = ((mx + my) * movementMultiplier - r) / denominator;
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
