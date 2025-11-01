# example_test
Example test for stonefish sim test setup.

- Shows how to use the test_utils package to launch stonefish simulator
in a test scenario.
- Shows how to modify the scenario config to apply test-specific scenario parameters.
- Shows how scenario config parameters can be extracted and used in the test code.

## Running the test

```bash
colcon test --packages-select example_test --event-handlers console_direct+
```