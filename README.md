# FRC-2019
604 Quixilver's 2019 Robot Code

## Setting up WPILib
More documentation and instructions avaliable at: https://docs.wpilib.org

Click the link below and dowload the installer for your OS.
https://github.com/wpilibsuite/allwpilib/releases

1.Follow the instructions for you operating system.
(You do not need to download the Update Suite)
  #### Windows
  a. Unzip the file by clicking Extract All
  
  b. Go into the unziped folder and run WPILibInstaller_Windows64-2019.1.1.exe(If it asks if you trst the app, click `More Info` then `Run`
  
  c. Install for All Users
  
  d. Click Slect/Download VSCode
  
  e. Click Download and wait for it to finish
  
  f. Make sure all check boxes are checked
  
  g. Click Execute Install
  
  #### MacOS
  a. Go to downloads and doubble click on the WPILib_Mac-2019.1.1.tar file.
 
 b. Go to https://code.visualstudio.com and download VSCode(Click Download for Mac)
 
 c. Doubble click on VSCode-darwin-stable.zip(It may automattically exctract and the file will be `Visual Studio Code.app`, you dont have to doubble click this)
 
 d. Drag the `Visual Studio Code.app` into Applications.
 
 e. Open  a new `Terminal` window and type in `mkdir frc2019`(This will make a new folder in your home directory called frc2019)
 
 f. Then type in `cd frc2019` this will run your terminal commands in that folder(directory)
 
 g. Now run all of these commands to copy the files, one at a time.
 
 ```
   cp -R ~/Downloads/WPILib_Mac-2019.1.1/documentation ~/frc2019
  
	cp -R ~/Downloads/WPILib_Mac-2019.1.1/frccode ~/frc2019
  
	cp -R ~/Downloads/WPILib_Mac-2019.1.1/installUtils ~/frc2019
  
	cp -R ~/Downloads/WPILib_Mac-2019.1.1/jdk ~/frc2019
  
	cp -R ~/Downloads/WPILib_Mac-2019.1.1/jdk-11.0.1.jdk ~/frc2019
  
	cp -R ~/Downloads/WPILib_Mac-2019.1.1/maven ~/frc2019
  
	cp -R ~/Downloads/WPILib_Mac-2019.1.1/roborio ~/frc2019
  
	cp -R ~/Downloads/WPILib_Mac-2019.1.1/tools ~/frc2019
  
	cp -R ~/Downloads/WPILib_Mac-2019.1.1/utility ~/frc2019
  
	cp -R ~/Downloads/WPILib_Mac-2019.1.1/vsCodeExtensions ~/frc2019
  ```
  
  (Run `ls` to make sure all the folders(direcotries) copied over)
	
 h. Now run `cd ~/frc2019/tools`
 
 i. Run `python ToolsUpdater.py` which runs the ToolsUpdater script.
 
 j. You can type `ls` into Terminal to make sure it worked(You should see `.py`, `.vbs` and `.jar` files)

 k. Close terminal and launch VSCode from applications.
 
 l. Hold `Command+Shift+P` to bring up possible commands.
 
 m. Type `VSIX` into the prompt, then click `Extensions:Install from VSIX`
 
 n. In the Finder prompt, locate the dropdown menu in the center of the window at the top and select the home icon/emoji.
 
 o. Then open the folder frc2019, then vsCodeExtensions.
 
 p.Select `Cpp.vsix` and then Install.
 
 q. It will take a second, in the bottom right of VSCode you will be prompted to reload, click `Reload Now`.
 
 r. Now repeat steps m-q for all the `.visx` files in theis order.
 ```
1. Cpp.vsix

2. JavaLang.vsix

3. JavaDeps.vsix

4. JavaDebug.vsix

5. WPILib.vsix
```
s. Finally, bring up the command palette(`Command+Shift+P`) and type in `Set VS Code Java Home to FRC Home`, then run that command by clicking it, Select `global`.

## Git

1. Download git for you operating system

 https://git-scm.com/downloads
 
2.Run the dowloaded installer
  (On MacOS douddle click on the `.pkg` file on the prompt(you make have to go to `Settings -> Secrity & Privacy ->Open prompt at the bottom of page` to open it)

Here are the options you should use when setting it up.

#### Windows
  a. First click next to accept the license.
  
  b. Click next on the Destination Location.
  
  c. Click next on Select Components.
  
  d. Click next on Select Start Menu Folder
  
  e. Slelect Vim editor for git, then select next.
  
  f. Choose Git from the command line and also from 3rd-party software.
  
  g. Choose to use the OpenSSl library.
  
  h. Choose Checkout Windows-style, commit Unix-style line endings.
  
  i.Choose to use MinTTY.
  
  j.Then click Install.
  
#### MacOs
  a. Click continue
  
  b. Click Install
  
  c. Enteryou adminstrator account username and passowrd.
  
  d. Choose to keep the package.
  
## Robot Code in VSCode

a. To start, make a folder somewhere to store your robot code, then open terminal or the command line and navigate into the folder.

`cd "Destination of Folder`

Ex(Windows). `cd Desktop/FRC_2019`

ex(MacOs). `cd /Desktop/FRC_2019`

b. Now you need to clone the repositry into the folder(like copy and pasting).

`git clone https://github.com/frc604/FRC-2019.git`

c. Now go into Visual Studio COde and click `File -> Open Folder` and click on the folder `FRC-2019` in the folder that you created.

## Pulling Certain Branch

a. `cd` into the the code repo, into the folder you made, then into FRC-2019

`cd "Folder Path/FRC-2019"`

b. Then type `git fetch` -v to list all the possibble branches.

c. Finally type `git checkout "Name of Branch"` to 2switch to that branch.


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

## Shuffleboard
1. First locate the FRC Shuffleboard shortcut on the desktop or the `shuffleboard.vbs` file.

2. Next open Shuffleboard and click `File -> Open Layout`

3. In the prompt locate your reposoirity location and select the `shuffleboard_FRC-2019.json` file in the `ShuffleboardFiles` folder.

## Limelight
### Flashing
When flashing the Limelight, follow the instructions found [here](http://docs.limelightvision.io/en/latest/getting_started.html#imaging).
However, it is also nessessary to install the drivers for a Raspberry Pi compute module as well.
You will know you have succeeded when the Pi shows up as a removable drive (if on Windows).

## Drivers
http://www.sapphiretech.com/product_downloadmore.asp?PID=1482&CataID=30&lang=eng
