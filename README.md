# FRC-2019
604 Quixilver's 2019 Robot Code

## Structure
The main source code can be found in `src/com/_604robotics/`

Custom libraries (Pathfinder) can be found in `lib/`

## Using Gradle
The official build system for FIRST FRC Robotics has officially been changed to gradle.
This means that there is no need to be stuck with any IDE or editor, and grants more freedom.
That being said, it does require the usage of terminal/console/CMD commands.

### Deploying
To deploy code to the robot, two things must happen.

1. When connected to the *internet*, **not** the robot, run `./gradlew downloadAll`
This will download all the needed dependencies for gradle and the project itself.
2. Before deploying, the code must be built with `./gradlew build`
This will create Jars of any code changes
3. Then, when connected to the *robot*, run `./gradlew deploy --offline`
This will push all *previously built* code to the robot

### Setting up editor
Currently, there are two IDEs that are added into the plugin list in `build.gradle`:
* Intellij IDEA
* Eclipse

When first cloning the repo, or after making changes to `build.gradle`, it is nessessary to generate new IDE configuration files
Depending on your IDE, the command is (respectively):
* `./gradlew idea`
* `./gradlew eclipse`

## Dependencies
This year's robot code uses [604's modified version of Pathfinder](https://github.com/frc604/Pathfinder), the original version of which can be found [here](https://github.com/JacisNonsense/Pathfinder).

Uses Pixy code adapted from [BHSRobotix/Steamworks2017](https://github.com/BHSRobotix/Steamworks2017)

## Documentation
The datasheet for the VL53L0X rangefinder can be found [here](https://www.st.com/content/ccc/resource/technical/document/user_manual/group0/6b/4e/24/90/d8/05/47/a5/DM00279088/files/DM00279088.pdf/jcr:content/translations/en.DM00279088.pdf).
