package frc.robot.autonomous;

import com.fasterxml.jackson.databind.MapperFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.genericrobot.PixyCam;
import lombok.Builder;
import lombok.Value;
import lombok.extern.jackson.Jacksonized;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import static frc.robot.autonomous.GalacticSearchDecision.Path.*;
import static frc.robot.autonomous.GalacticSearchDecision.*;
import static org.junit.Assert.*;

public class GalacticSearchControlTest {
	public Data data;
	public static @Jacksonized @Builder @Value class Data {
		public PixyCam.Block[] REDA,REDB,BLUA,BLUB,TooMany,TooFew;
	}

	@Before
	public void setUp() throws Exception {
		data = new ObjectMapper()
			.configure(MapperFeature.ACCEPT_CASE_INSENSITIVE_PROPERTIES, true)
			.readValue(this.getClass().getClassLoader().getResource("pixycamData.json"), Data.class);
	}

	@Test public void redA() {
		assertEquals(RedA, parsePath(data.REDA));
	}

	@Test public void redB() {
		assertEquals(RedB, parsePath(data.REDB));
	}

	@Test public void blueA() {
		assertEquals(BlueA, parsePath(data.BLUA));
	}

	@Test public void blueB() {
		assertEquals(BlueB, parsePath(data.BLUB));
	}

	@Test public void nullT() {
		assertEquals(Unknown, parsePath(null));
	}

	@Test public void empty() {
		assertEquals(Unknown, parsePath(new PixyCam.Block[0]));
	}
	@Test public void tooFew() {
		assertEquals(Unknown, parsePath(data.TooFew));
	}
	@Test public void tooMany() {assertEquals(Unknown, parsePath(data.TooMany));}

	@Test public void queueMode() {
		GalacticSearchDecision ctrl = new GalacticSearchDecision();
		assertEquals(Unknown, ctrl.readAndChoosePath( RedA));
		assertEquals(Unknown, ctrl.readAndChoosePath( RedA));
		assertEquals(Unknown, ctrl.readAndChoosePath( RedA));
		assertEquals(Unknown, ctrl.readAndChoosePath( RedA));
		assertEquals(Unknown, ctrl.readAndChoosePath( RedA));
		assertEquals(Unknown, ctrl.readAndChoosePath( RedB));
		assertEquals(Unknown, ctrl.readAndChoosePath( RedB));
		assertEquals(Unknown, ctrl.readAndChoosePath( RedB));
		assertEquals(Unknown, ctrl.readAndChoosePath( RedB));
		assertEquals(RedA   , ctrl.readAndChoosePath(BlueA));
		//replace a redA with a redB, assert dominance
		assertEquals(RedB   , ctrl.readAndChoosePath( RedB));
	}

}