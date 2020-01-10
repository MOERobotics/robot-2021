package frc.robot.genericrobot;

public abstract class GenericRobot {

      public double         leftPower = 0;
      public double        rightPower = 0;
      public double         spinPower = 0;
      public double shooterUpperPower = 0;
      public double shooterLowerPower = 0;

      public final void setMotorPowerPercentage(
            double leftPower,
            double rightPower
      ) {
            this. leftPower =  leftPower;
            this.rightPower = rightPower;
            setMotorPowerPercentageInternal(
                  leftPower,
                  rightPower
            );
      }

      protected abstract void setMotorPowerPercentageInternal(
            double leftPower,
            double rightPower
      );

      public final void driveForward (
            double power
      ) {
            setMotorPowerPercentage(power,power);
      }

      public final void driveReverse (
            double power
      ) {
            setMotorPowerPercentage(-power,-power);
      }

      public final void driveLeftInPlace (
            double power
      ) {
            setMotorPowerPercentage(-power,power);
      }

      public final void driveRightInPlace (
            double power
      ) {
            setMotorPowerPercentage(power,-power);
      }

      public final double getMotorPowerLeft() {
            return leftPower;
      }

      public final double getMotorPowerRight() {
            return rightPower;
      }

      public final double getDistanceInchesLeft()  {
            return getDistanceTicksLeft() * getDistanceRatioLeft();
      }
      public double getDistanceRatioLeft()  {
            System.out.println("I don't have an encoder :'(");
            return 0;
      }
      public double getDistanceTicksLeft()  {
            System.out.println("I don't have an encoder :'(");
            return 0;
      }

      public double getDistanceInchesRight()  {
            return getDistanceTicksRight() * getDistanceRatioRight();
      }
      public double getDistanceRatioRight()  {
            System.out.println("I don't have an encoder :'(");
            return 0;
      }
      public double getDistanceTicksRight()  {
            System.out.println("I don't have an encoder :'(");
            return 0;
      }

      public double getYaw()  {
            System.out.println("I don't have a navx :'(");
            return 0;
      }

      public double getPitch()  {
            System.out.println("I don't have a navx :'(");
            return 0;
      }

      public double getRoll()  {
            System.out.println("I don't have a navx :'(");
            return 0;
      }


      public final void setShooterPowerPercentage(
            double upperPower,
            double lowerPower
      ) {
            this.shooterUpperPower = upperPower;
            this.shooterLowerPower = lowerPower;
            setShooterPowerPercentageInternal(
                    upperPower,
                    lowerPower
            );
      }

      public final void setShooterPowerPercentage(
            double power
      ) {
            setShooterPowerPercentageInternal(power,power);
      }

      protected void setShooterPowerPercentageInternal(
            double upperPower,
            double lowerPower
      ) {
            System.out.println("I don't have a shooter :'(");
      }

      public final double getShooterPowerUpper() {
            return shooterUpperPower;
      }
      public final double getShooterPowerLower() {
            return shooterLowerPower;
      }


      public final void spinControlPanel (
            double power
      ) {
            this.spinPower = power;
            spinControlPanelInternal(power);
      }

      protected void spinControlPanelInternal (
            double power
      ) {
            System.out.println("I can't spin the control panel :'(");
      }

      public final double getControlPanelSpinnerPower () {
            return spinPower;
      }

      public char getCurrentControlPanelColor()  {
            System.out.println("I don't have a color sensor :'(");
            return '?';
      }

      public double getLimelightX(){
            System.out.println("I don't have a limelight ;(");
            return 0;
      }

      public double getLimelightY(){
            System.out.println("I don't have a limelight ;(");
            return 0;
      }

      public double getLimelightArea(){
            System.out.println("I don't have a limelight ;(");
            return 0;
      }
}
