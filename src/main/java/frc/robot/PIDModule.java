package frc.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;



public class PIDModule{
    public double getInput      () {return  input; }
    public double getCorrection () {return output; }
    public void   setHeading    (double input) {this.input = input;  }

    public void resetError(){
        this.input = 0;
        this.pidController.reset();
        this.pidController.enable();
    }

    private double input;
    private double output;

    public final double kP, kI, kD;
    public final PIDController pidController;

    public PIDModule (
            double kP,
            double kI,
            double kD
    ) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        PIDModuleInternal internal = new PIDModuleInternal(this);

        this.pidController = new PIDController(
                kP,
                kI,
                kD,
                internal,
                internal
        );
        this.pidController.setInputRange(-180,180);
        this.pidController.setOutputRange(-1,1);
        this.pidController.setContinuous(true);
        this.pidController.reset();
        this.pidController.enable();
    }

    private class PIDModuleInternal implements PIDSource, PIDOutput{
        PIDModule parent;
        PIDModuleInternal(PIDModule parent){
            this.parent = parent;
        }

        @Override
        public void pidWrite(double output) {
            parent.output = output;
        }

        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {

        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return PIDSourceType.kDisplacement;
        }

        @Override
        public double pidGet() {
            return parent.input;
        }
    }



}
