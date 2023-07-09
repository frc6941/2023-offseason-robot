import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;

import frc.robot.Ports;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.State;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class IndexerTest {
    static final double DT = 0.02;
    double currentTime = 0.0;
    Indexer indexer = Indexer.getInstance();
    AnalogInputSim bottomSim = new AnalogInputSim(Ports.AnalogInputId.BOTTOM_BEAM_BREAK_CHANNEL);
    AnalogInputSim topSim = new AnalogInputSim(Ports.AnalogInputId.TOP_BEAM_BREAK_CHANNEL);

    void setOn(AnalogInputSim sim) {
        sim.setVoltage(0.0);
    }

    void setOff(AnalogInputSim sim) {
        sim.setVoltage(5.0);
    }

    void elapseLoop(int loopCount) {
        for (int i = 0; i < loopCount; i++) {
            currentTime += DT;
            indexer.read(currentTime, DT);
            indexer.update(currentTime, DT);
        }
    }

    void elapseTime(double dt) {
        elapseLoop((int) Math.ceil(dt / DT));
    }

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
        setOff(topSim);
        setOff(bottomSim);
        currentTime = 0.0;
    }

    @Test
    void test2CorrectBallIndexIdealSequence() {
        assert indexer.getBallCount() == 0;

        // 1. a correct ball is detected
        indexer.queueBall(true);
        elapseTime(0.02);
        assert indexer.getState() == State.INDEXING;
        assert indexer.getBallCount() == 0; // processing ball does not count to ball count

        // 2. the correct ball reached the top
        elapseTime(0.50);
        setOn(topSim);
        assert indexer.getState() == State.INDEXING; // still need to be indexing, as the ball is not nested
        elapseTime(0.20);
        assert indexer.getState() == State.IDLE; // the indexer should stop shortly after
        assert indexer.getBallCount() == 1;

        // 3. the second ball is detected
        indexer.queueBall(true);
        elapseTime(0.20);
        assert indexer.getState() == State.INDEXING;
        assert indexer.getBallCount() == 1; // processing ball does not count to ball count

        // 4. the correct ball reached the top
        elapseTime(0.50);
        setOn(bottomSim);
        assert indexer.getState() == State.INDEXING; // still need to be indexing, as the ball is not nested
        elapseTime(0.20);
        assert indexer.getState() == State.IDLE; // the indexer should stop shortly after
        assert indexer.getBallCount() == 2;

        // 5. when the ballpath is full, indexer will not queue and run
        indexer.queueBall(true);
        assert indexer.getState() == State.IDLE;
    }

    @Test
    void testAlternatingBallIndex() {

        assert indexer.getBallCount() == 0;

        // 1. a wrong ball is detected and ejected
        setOff(bottomSim);
        indexer.queueBall(false);
        elapseTime(1.00);
        assert indexer.getState() == State.EJECTING; // stil ejecting as exit is not reached
        setOn(bottomSim); // reach the exit
        elapseTime(0.10);
        assert indexer.getState() == State.EJECTING;
        setOff(bottomSim); // should start the ejected timer
        elapseTime(0.20);
        assert indexer.getState() == State.IDLE; // stop after timeout

        // 2. a correct ball enters
        elapseTime(0.50);
        indexer.queueBall(true);
        elapseTime(0.20);
        setOn(topSim);
        assert indexer.getState() == State.INDEXING; // still need to be indexing, as the ball is not nested
        elapseTime(0.20);
        assert indexer.getState() == State.IDLE; // the indexer should stop shortly after
        assert indexer.getBallCount() == 1;

        // 3. a wrong ball enters with a correct one
        elapseTime(0.50);
        indexer.queueBall(false);
        elapseTime(0.50);
        indexer.queueBall(true);
        elapseTime(0.02);
        assert indexer.getState() == State.EJECTING;

        elapseTime(0.50);
        assert indexer.getState() == State.EJECTING; // the indexer should eject the wrong ball now
        setOn(bottomSim);
        elapseTime(0.20);
        setOff(bottomSim);
        elapseTime(1.00);
        assert indexer.getState() == State.INDEXING; // after eject should index

        setOn(bottomSim); // reach cargo bottom slot
        elapseTime(0.10);
        assert indexer.getState() == State.IDLE; // should stop
    }

    @Test
    void testIntakingWhileFeeding() {
        indexer.setWantFeed(true);
        elapseTime(0.02);
        assert indexer.getState() == State.FEEDING;

        indexer.queueBall(false);
        elapseTime(1.00);
        assert indexer.getState() == State.EJECTING; // stil ejecting as exit is not reached

        indexer.queueBall(true);
        setOn(bottomSim);
        elapseTime(0.10);
        assert indexer.getState() == State.EJECTING;
        setOff(bottomSim); // should start the ejected timer
        elapseTime(0.20);
        assert indexer.getState() == State.FEEDING; // continue feeding after timeout
        assert indexer.getBallCount() == 0;

        indexer.queueBall(true);
        setOn(topSim);
        assert indexer.getBallCount() == 0;
        assert indexer.getState() == State.FEEDING;
    }

    @Test
    void testForceEject() {
        indexer.setWantForceEject(true);
        elapseTime(0.02);
        assert indexer.getState() == State.FORCE_EJECTING;

        indexer.setWantForceEject(false);
        elapseTime(0.02);
        assert indexer.getState() == State.FORCE_EJECTING; // need to still be in force ejecting mode to clear ballpath

        elapseTime(1.0);
        assert indexer.getState() == State.IDLE;
    }

    @Test
    void testForceReverse() {
        indexer.setWantForceReverse(true);
        elapseTime(0.02);
        assert indexer.getState() == State.FORCE_REVERSING;

        indexer.setWantForceReverse(false);
        elapseTime(0.02);
        assert indexer.getState() == State.FORCE_REVERSING; // need to still be in force ejecting mode to clear ballpath

        elapseTime(1.0);
        assert indexer.getState() == State.IDLE;
    }

    @AfterEach
    void shutdown() {
        indexer.reset();
    }
}