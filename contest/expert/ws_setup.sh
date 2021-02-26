#!/bin/bash

echo "###############################################################################"
echo "Workshop environment setup starting.."
echo "###############################################################################"

# Wait if apt is running. 
while :
do
    count=`ps -ef | grep apt.systemd.daily | grep lock_is_held | grep -v grep | wc -l`
    if [ $count = 0 ]; then
        break
    else
        echo "System update is running.. Wait until the complete"
        sleep 10
    fi
done

sudo apt-get update
source /opt/ros/$ROS_DISTRO/setup.sh
rosdep update

pip3 install -U awscli
pip3 install -U colcon-common-extensions colcon-ros-bundle colcon-bundle
pip3 install boto3

STACK_NAME1=deliverychallenge`echo $C9_USER|tr -d [].\\\-\\\=_/` 
STACK_NAME2=deliverychallengecognito`echo $C9_USER|tr -d [].\\\-\\\=_/` 

aws cloudformation deploy --template-file ../../cf_templates/bootstrap.cfn.yaml --stack-name $STACK_NAME1 --capabilities CAPABILITY_NAMED_IAM
aws cloudformation deploy --template-file ../../cf_templates/cognito_setting.cfn.yaml --stack-name $STACK_NAME2 --capabilities CAPABILITY_IAM

../../install_utils/setup.bash
../../setup_ROBOTIS_sample.sh

curl -o ./simulation_ws/bundle/output.tar https://deliverychallenge2021-assets.s3-ap-northeast-1.amazonaws.com/setup/simulation.tar

cd ./robot_ws
rosws update 

cd ..

curl -o ./robot_ws/src/aws_game_manager/certs/AmazonRootCA1.pem https://www.amazontrust.com/repository/AmazonRootCA1.pem

python3 ./ws_setup.py $STACK_NAME1 $STACK_NAME2

if [ -f ~/environment/roboMakerSettings.json ]; then
    cp ./roboMakerSettings.json ~/environment/roboMakerSettings.json
fi
