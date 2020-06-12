
  const awsIot = require('aws-iot-device-sdk');


  async function main() {

      // FIXME: 直接Credentialを指定する場合に有効にする ->
      //const credentials = await getRawCredentials();
      
      // <-
      async function getCognitoCredentials() {
        AWS.config.region = region;
        var cognitoidentity = new AWS.CognitoIdentity();
        var params = {
          IdentityPoolId: PoolId
        };
        const identityId = await cognitoidentity.getId(params).promise();
        const data = await cognitoidentity.getCredentialsForIdentity(identityId).promise();
            var credentials = {
              accessKey: data.Credentials.AccessKeyId,
              secretKey: data.Credentials.SecretKey,
              sessionToken: data.Credentials.SessionToken
            };
            return credentials;
      }
      // FIXME: Cognitoを使ってCredentialを取得する場合に有効にする ->
      const credentials = await getCognitoCredentials();
      
      // <-

      const deviceIot = awsIot.device({
          region: region,
          clientId: iotclientId,
          accessKeyId: credentials.accessKey,
          secretKey: credentials.secretKey,
          sessionToken : credentials.sessionToken, // FIXME: 直接Credentialを指定する場合はコメントアウト
          protocol: 'wss',
          port: 443,
          host: iotendpoint
      });

      deviceIot.on('message', function(_topic, payload) {
          var json = JSON.parse(payload.toString());
          command = json["command"];
          if (command == "location") {
            odom = json["odom"];
            vue.ODOM_X =  odom["x"].toFixed(4);
            if (odom) {
              vue.ODOM_X =  odom["x"].toFixed(4);
              vue.ODOM_Y =  odom["y"].toFixed(4);
              vue.ODOM_Z =  odom["z"].toFixed(4);
              vue.ODOM_ORIENTATION =  (odom["yaw"] * ( 180 / Math.PI )).toFixed(4);
            }
          }
          else if (command == "result") {
            alert(json["message"])
          }
      });

      deviceIot.subscribe(subscribe_topic, undefined, function (err, granted){
          if( err ){
              console.log('subscribe error: ' + err);
          } else {
              console.log('subscribe success');
          }
      });

      //----
      
      setInterval(syncJob, 1000);

      function syncJob() {
        let payload = {};
        let shouldPublish = false;
        if (move_action !== "") {
          console.log(move_action);
          payload["command"] = "move";
          payload["action"] = move_action;
          shouldPublish = true;

        } 
        if (shouldPublish) {
          deviceIot.publish(publish_topic, JSON.stringify(payload));
        }
      }

      let move_action = "";

      let vue = new Vue({
        el: '#app',
        data: {
            ODOM_X:0,
            ODOM_Y:0,
            ODOM_Z:0,
            ODOM_ORIENTATION:0,
            target_x:0,
            target_y:0,
            target_orientation:0,
        },        
      methods: {
        move_forward_down: function () {console.log("move_forward pressed"); move_action = "forward"; syncJob()},
        move_forward_up: function () {console.log("move_forward released"); move_action = "";},
        move_left_down: function () {console.log("move_left pressed"); move_action = "left"; syncJob()},
        move_left_up: function () {console.log("move_left released"); move_action = "";},
        move_right_down: function () {console.log("move_right pressed"); move_action = "right"; syncJob()},
        move_right_up: function () {console.log("move_right released"); move_action = "";},
        move_backward_down: function () {console.log("move_backward pressed"); move_action = "backward"; syncJob()},
        move_backward_up: function () {console.log("move_backward released"); move_action = "";},
        move_stop_down: function () {console.log("move_stop pressed"); move_action = "stop"; syncJob()},
        move_stop_up: function () {console.log("move_stop released"); move_action = "";},
        map_save: function () {
          let payload = {};
          console.log("save map!!");
          request_id =  (new Date()).getTime();
          payload["command"] = "map";
          payload["action"] = "save";
          payload["request_id"] = request_id
          console.log(payload);
          deviceIot.publish(publish_topic, JSON.stringify(payload));
        },
        go_to_target: function() {
          console.log("go to target!!");
          let payload = {};
          request_id =  (new Date()).getTime();
          payload["command"] = "navigation";
          payload["action"] = "setGoal";
          payload["request_id"] = request_id
          payload["x"] = Number(this.target_x);
          payload["y"] = Number(this.target_y);
          payload["yaw"] = this.target_orientation * ( Math.PI / 180 ) ;
          console.log(payload);
          deviceIot.publish(publish_topic, JSON.stringify(payload));
        }
      }
    })

    }

  main();
