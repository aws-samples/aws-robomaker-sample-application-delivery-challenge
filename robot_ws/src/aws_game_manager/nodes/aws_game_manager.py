#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#

# 
# このノードは ROS アプリケーションと AWS IoTとを連携し、ロボットをコントロールします
# AWS Robot Delivery Challenge の予選リーグ参加者はこのファイルを変更しないでください。
# 

import rospy
from std_msgs.msg import String
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
import argparse
import json
import rospkg
import yaml
import sys
import requests
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
import math
import time
import threading
import os

class GameManager:
    
    def __init__(self, aws_iot_client, config):
        
        self._iot_client = aws_iot_client
        self._iot_client.set_game_command_cb(self._gm_command_cb)

        self._isSimulation = rospy.get_param("use_sim_time")
        self._config = config

        if self._isSimulation:
            try:
                self._gm_cmd = rospy.Publisher('/gm_cmd', String)
                rospy.Subscriber('/gm_status', String, self._game_status_cb)               
            except Exception as e:
                self._status = 'Failure'
                rospy.logwarn(e)
                
        
    def main(self):
        while not rospy.is_shutdown() :
            try:
                self._update_credentials()
            except Exception as e:
                rospy.logerr("Failed to update AWS IoT credentail")
                rospy.logerr(e)
                time.sleep(10)
                continue

            time.sleep(60 * 30)
    
    def _update_credentials(self):
        rospy.loginfo("Update AWS IoT Credentials")
        url = "https://{}:443/role-aliases/{}/credentials".format(self._config["credentialEndpoint"], self._config["roleAlias"])
        folder_path = self._config['configFilePath']        
        res = requests.get(url,
            verify=os.path.join(folder_path, self._config["rootCAFile"]),
            cert=(os.path.join(folder_path, self._config["certFile"]), 
                    os.path.join(folder_path, self._config["privateKeyFile"])))

        if res.status_code != requests.codes.ok:
            rospy.loginfo("Update AWS IoT Credentials fail")
            rospy.logwarn(res)
            return False

        credential_json = res.json()
        credentials = credential_json["credentials"]
        rospy.set_param("/aws/iot/credential/accessKeyId", credentials["accessKeyId"])
        rospy.set_param("/aws/iot/credential/secretAccessKey", credentials["secretAccessKey"])
        rospy.set_param("/aws/iot/credential/sessionToken", credentials["sessionToken"])
        rospy.set_param("/aws/iot/credential/expiration", credentials["expiration"])

        rospy.loginfo("Update AWS IoT Credentials finished")
        return True
    
    def _gm_command_cb(self, payload):
        self._gm_cmd.publish(json.dumps(payload))

    def _game_status_cb(self, message):
        self._iot_client.gm_to_awsiot_publisher(message.data)
        

class MqttRos:
    AllowedActions = ['both', 'publish', 'subscribe']

    def __init__(self, config):
        self.iot_data = config
        self.thing_name = self.iot_data["thingName"]
        self.subscribe_topic = self.iot_data["subscribeTopic"]
        self.publish_topic = self.iot_data["publishTopic"]
        self.client_id = self.thing_name + '_mqtt'

        self.init_mqtt_client()
        self.init_ros_pubs()
        self.init_ros_subs()
        self.mqtt_subs()
                
    def init_ros_pubs(self):
        # Place holder publisher into ros space.
        self.mqttToRosPub = rospy.Publisher('awsiot_to_ros', String, queue_size=1)

    def init_ros_subs(self):
        self.rosPubToMqtt = rospy.Subscriber('ros_to_awsiot', String, self.ros_to_mqtt_cb, queue_size=10)

    def ros_to_mqtt_cb(self, msg):
        self.ros_to_awsiot_publisher(msg)

    def ros_to_awsiot_publisher(self, msg):
        try:
            self.myAWSIoTMQTTClient.publish(self.publish_topic, msg.data, 1)
        except Exception as e:
            rospy.logwarn("MqttRos::ros_to_mqtt_cb got exception")
            rospy.logwarn(e)

    def gm_to_awsiot_publisher(self, message):
        try:
            self.myAWSIoTMQTTClient.publish('gm_{}'.format(self.publish_topic), str(message), 1)
        except Exception as e:
            rospy.logwarn(e)
            rospy.logwarn("MqttRos::gm_publisher got exception")

    #  MQTT message callback
    def mqtt_callback(self, client, userdata, message):
        try:
            mqttToRosJson = {}
            mqttToRosJson['payload'] = json.loads(message.payload)
            mqttToRosJson['topic'] = message.topic
            self.mqttToRosPub.publish(json.dumps(mqttToRosJson))
        except Exception as e:
            rospy.logwarn("MqttRos::mqtt_callback got exception")

    def gm_mqtt_callback(self, client, userdata, message):
        try:
            payload = json.loads(message.payload)
            self._game_command_handler(payload)
        except Exception as e:
            rospy.logwarn(e)
            rospy.logwarn("MqttRos::gm_mqtt_callback got exception")

    def init_mqtt_client(self):
        # Grab all required info from the parsed data
        folder_path = self.iot_data['configFilePath']

        host = self.iot_data['endpoint']
        rootCAPath = os.path.join(folder_path, self.iot_data['rootCAFile'])
        certificatePath = os.path.join(folder_path, self.iot_data['certFile'])
        privateKeyPath = os.path.join(folder_path, self.iot_data['privateKeyFile'])
        useWebsocket = self.iot_data['useWebsocket']
        self.mode = self.iot_data['mqttMode']

        if self.mode not in MqttRos.AllowedActions:
            rospy.logwarn("Unknown --mode option %s. Must be one of %s" % (self.mode, str(MqttRos.AllowedActions)))
            exit(2)
        if useWebsocket and certificatePath and privateKeyPath:
            rospy.logwarn("X.509 cert authentication and WebSocket are mutual exclusive. Please pick one.")
            exit(2)
        if not useWebsocket and (not certificatePath or not privateKeyPath):
            rospy.logwarn("Missing credentials for authentication.")
            exit(2)

        if useWebsocket:
            port = 443
        if not useWebsocket:
            port = 8883


        # Init AWSIoTMQTTClient
        self.myAWSIoTMQTTClient = None
        if useWebsocket:
            self.myAWSIoTMQTTClient = AWSIoTMQTTClient(self.client_id, useWebsocket=True)
            self.myAWSIoTMQTTClient.configureEndpoint(host, port)
            self.myAWSIoTMQTTClient.configureCredentials(rootCAPath)
        else:
            self.myAWSIoTMQTTClient = AWSIoTMQTTClient(self.client_id)
            self.myAWSIoTMQTTClient.configureEndpoint(host, port)
            self.myAWSIoTMQTTClient.configureCredentials(rootCAPath, privateKeyPath, certificatePath)

        # AWSIoTMQTTClient connection configuration
        self.myAWSIoTMQTTClient.configureAutoReconnectBackoffTime(1, 32, 20)
        self.myAWSIoTMQTTClient.configureOfflinePublishQueueing(-1)  # Infinite offline Publish queueing
        self.myAWSIoTMQTTClient.configureDrainingFrequency(2)  # Draining: 2 Hz
        self.myAWSIoTMQTTClient.configureConnectDisconnectTimeout(10)  # 10 sec
        self.myAWSIoTMQTTClient.configureMQTTOperationTimeout(5)  # 5 sec

    def mqtt_subs(self):
        # Connect and subscribe to AWS IoT
        self.myAWSIoTMQTTClient.connect()
        if self.mode == 'both' or self.mode == 'subscribe':
            self.myAWSIoTMQTTClient.subscribe(self.subscribe_topic, 1, self.mqtt_callback)

        self.myAWSIoTMQTTClient.subscribe('gm_{}'.format(self.subscribe_topic), 1, self.gm_mqtt_callback)

    def set_game_command_cb(self, callback):
        self._game_command_handler = callback


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: aws_game_manager.py confing_file_name")
        exit(1)
        
    rospy.init_node('aws_game_manager')

    config = {}
    config_file = rospy.get_param("~aws_iot_config_file")
    if config_file == "use_default":
        config_file = rospy.get_param("~aws_iot_config_file_default")

    with open(config_file, 'r') as f:
        try:
            config = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            rospy.logerr("yaml read error")
    config["configFilePath"] = os.path.dirname(config_file)

    aws_iot_client = MqttRos(config)
    game_manager = GameManager(aws_iot_client, config)
    game_manager.main()
