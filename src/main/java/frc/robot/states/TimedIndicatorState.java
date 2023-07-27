package frc.robot.states;

import edu.wpi.first.wpilibj.util.Color;

/**
 * From Team 254.
 */
public interface TimedIndicatorState {
    void getCurrentIndicatorState(IndicatorState desiredState, double timestamp);

    class BlinkingIndicatorState implements TimedIndicatorState {
        IndicatorState mStateOne = new IndicatorState(0.0, 0.0, 0.0);
        IndicatorState mStateTwo = new IndicatorState(0.0, 0.0, 0.0);
        double mDuration;

        public BlinkingIndicatorState(IndicatorState stateOne, IndicatorState stateTwo, double duration) {
            mStateOne.copyFrom(stateOne);
            mStateTwo.copyFrom(stateTwo);
            mDuration = duration;
        }

        @Override
        public void getCurrentIndicatorState(IndicatorState desiredState, double timestamp) {
            if ((int) (timestamp / mDuration) % 2 == 0) {
                desiredState.copyFrom(mStateOne);
            } else {
                desiredState.copyFrom(mStateTwo);
            }
        }
    }

    class IntervalBlinkingIndicatorState implements TimedIndicatorState {
        IndicatorState mStateOne = new IndicatorState(0.0, 0.0, 0.0);
        IndicatorState mStateTwo = new IndicatorState(0.0, 0.0, 0.0);
        IndicatorState mStateInterval = new IndicatorState(0.0, 0.0, 0.0);
        double blinkDuration;
        double blinkCount;
        double intervalDuration;

        public IntervalBlinkingIndicatorState(IndicatorState stateOne, IndicatorState stateTwo, IndicatorState stateInterval, double blinkDuration, double blinkCount, double intervalDuration) {
            mStateOne.copyFrom(stateOne);
            mStateTwo.copyFrom(stateTwo);
            mStateInterval.copyFrom(stateInterval);
            this.blinkDuration = blinkDuration;
            this.blinkCount = blinkCount;
            this.intervalDuration = intervalDuration;
        }

        @Override
        public void getCurrentIndicatorState(IndicatorState desiredState, double timestamp) {
            double cycleTime = blinkDuration * blinkCount * 2;
            double currentProgress = timestamp % (cycleTime + intervalDuration);
            if (currentProgress <= cycleTime) {
                if ((int) (currentProgress / blinkDuration) % 2 == 0) {
                    desiredState.copyFrom(mStateOne);
                } else {
                    desiredState.copyFrom(mStateTwo);
                }
            } else {
                desiredState.copyFrom(mStateInterval);
            }
        }
    }

    class StaticIndicatorState implements TimedIndicatorState {
        IndicatorState mStaticState = new IndicatorState(0.0, 0.0, 0.0);

        public StaticIndicatorState(IndicatorState staticState) {
            mStaticState.copyFrom(staticState);
        }

        @Override
        public void getCurrentIndicatorState(IndicatorState desiredState, double timestamp) {
            desiredState.copyFrom(mStaticState);
        }
    }

    class RainbowIndicatorState implements TimedIndicatorState {
        double mCycleTime;

        public RainbowIndicatorState(double cycleTime) {
            mCycleTime = cycleTime;
        }

        @Override
        public void getCurrentIndicatorState(IndicatorState desiredState, double timestamp) {
            double hue = (timestamp % mCycleTime) / mCycleTime * 180.0;
            desiredState.copyFrom(IndicatorState.createFromHSV(hue, 255, 255));
        }
    }

    class BreathingIndicatorState implements TimedIndicatorState {
        double mCycleTime;
        IndicatorState targetState;

        public BreathingIndicatorState(IndicatorState state, double cycleTime) {
            mCycleTime = cycleTime;
            targetState = state;
        }

        @Override
        public void getCurrentIndicatorState(IndicatorState desiredState, double timestamp) {
            double value = Math.abs(Math.sin(Math.PI / mCycleTime * (timestamp % mCycleTime)));
            desiredState.copyFrom(IndicatorState.createFromColor(new Color(targetState.red * value, targetState.green * value, targetState.blue * value)));
        }
    }

}
