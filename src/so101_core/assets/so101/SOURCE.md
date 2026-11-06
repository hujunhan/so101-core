# Source

Copied from [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
(`Simulation/SO101/`, commit `a758567c3978dfeefe282ede0500085a48fe8f78`), licensed under
Apache-2.0. Files are unmodified; only the follower "new calibration" model and the meshes it
uses are included.

"New calibration": each joint's zero is the middle of its range. This matches LeRobot's
`DEGREES` normalization, whose zero is the middle of the range recorded at calibration.
