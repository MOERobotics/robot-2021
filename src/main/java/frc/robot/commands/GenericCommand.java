package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class GenericCommand {

	protected int commandStep = 0;
	public final void printSmartDashboard() {
		SmartDashboard.putNumber ("Command Step"   ,               commandStep);
		SmartDashboard.putString ("Command Loaded" , this.getClass().getName());
		printSmartDashboardInternal();
	}
	protected void printSmartDashboardInternal() {}

	public void begin() {
		System.out.println("I don't define begin() steps in my command :'(");
	}

	public void step() {
		System.out.println("I don't define begin() steps in my command :'(");
	}

	//LOCK***************************************************
	private boolean locksControls = true;
	public final void setControlLock(boolean doesLock) {
		this.locksControls = doesLock;
	}
	public final boolean locksControls() {
		return this.locksControls;
	}

	//ENABLE***************************************************
	private boolean enabled = false;
	public final void setEnabled(boolean isEnabled) {
		this.enabled = isEnabled;
	}
	public final boolean isEnabled() {
		return this.enabled;
	}


	//DO_NOTHING_COMMAND**************************************
	public static GenericCommand doNothingCommand = new DoNothing();
	private static class DoNothing extends GenericCommand {
		public DoNothing() {
			setControlLock(false);
		}
		@Override public void begin() {
			setEnabled(false);
		}
		@Override public void step () {
			setEnabled(false);
		}
	}
}
