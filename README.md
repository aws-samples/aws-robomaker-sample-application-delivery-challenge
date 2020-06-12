
# Create map and navigate robot to designated location in RoboMaker simulator

This is open source version of the ROS application used in [AWS Robot Delivery Challenge](https://aws.amazon.com/jp/robot-delivery-challenge/) 

日本語のドキュメントが次にあります ([基礎](docs/HOWTO.md), [Linux](docs/HOWTO_Linux.md), [詳細](docs/Detail.md))

## Setup

You can easily setup the working environment on AWS RoboMaker development environment.

1. Create [RoboMaker development environment](https://console.aws.amazon.com/robomaker/home#ides). Kinetic and Melodic are supported for Pre-installed ROS distribution.

2. Clone the source code through the terminal window of the development environment.
    ```
    git clone https://github.com/aws-samples/aws-robomaker-sample-application-delivery-challenge.git
    ```

3. On the same terminal window, execute following commands.
    ```bash
    cd ~/environment
    cd aws-robomaker-sample-application-delivery-challenge
    ./ws_setup.sh
    ```

4. After all the setup process completed (it takes around 20-30 min), download the browser folder under aws-robomaker-sample-application-delivery-challenge directory to your local file system. You can do this by right click the browser folder and select [**Download**] from the context menu.

## How to use

This sample application provides two functionalities. One is creating map and the other is navigating the world. Let's do creating map first.

### Create map

1. From RoboMaker development environment, select menu *Run* ->  *Launch Simulation* -> *Delivery Challenge controller*
(reload browser if you don't see the menu items)

2. CLick [**Connect**] button of [**Gazebo**]. It shows the simulation world.

3. Click [**Connect**] button of [**Terminal**] in **Robot application tools**. Type following command in the terminal. It will bring the visualize tool for the robot's sensing data.

    ```bash
    export TURTLEBOT3_MODEL=burger
    roslaunch delivery_robot_sample slam.launch
    ```

    ![gazebo_rviz](docs/img/gazebo_and_rviz.png)

    From browser folder you download, find fine named *robot_controller.html*. Open the file with your web browser, it shows remote controller interface.

    ![browser](docs/img/browser_control.png)

    By moving the robot by pressing arrow buttons in the controller, you can build the map of the world. (The generated map is displayed in the terminal window)

    When you are fine for the map, press [Save Map] button in the remote controller interface. The map file will then be saved into Amazon S3 bucket. The bucket the map is stored is described in the *robot_ws/src/delivery_robot_sample/settings/settings.yaml file*.

 4. Cancel the simulation by selecting *Actions* -> *Cancel* from the detail page of the simulation job.
    
### Navigate the world

1. From RoboMaker development environment, select menu *Run*]* ->  *Launch Simulation* -> *Delivery Challenge navigation*

2. CLick [**Connect**] button of [**Gazebo**]. It shows the simulation world.

3. Click [**Connect**] button of [**Terminal**] in **Robot application tools**. Type following command in the terminal. It will bring the visualize tool for the robot's sensing data.(Note that the command is different from what you executed on Create map section. We are now visualizing different information)

    ```bash
    export TURTLEBOT3_MODEL=burger
    roslaunch delivery_robot_sample rviz_for_navigation.launch
    ```

      ![rviz_nav](docs/img/rviz_nav.png)

    Open the remote controller interface again. Set the designated location like x: 4, y: 1.4, heading: 0 in Goal section of the controller. Click [Go To] button, then. The robot will then create the navigation route to go to the location and the robot auto drive to the location.

Robot may stuck in the middle to avoid to collide the obstacles. It's time for tuning. You can tune the navigation parameters. They are in 
 **robot_ws/src/turtlebot3/turtlebot3_navigation/param**.
About the tuning points, you can find them from http://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#tuning-guide
When you make changes on your robot application, you should reflect the change to the application. You can do it by selecting, *Run* -> *Workflow* -> *Delivery Challenge Robot app build -> bundle* from the menu.

## License

This library is licensed under the MIT-0 License. See the LICENSE file.

