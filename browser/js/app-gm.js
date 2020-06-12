
  const awsIot = require('aws-iot-device-sdk');


  async function main() {

    const gm_subscribe_topic = "gm_" + subscribe_topic;
    const gm_publish_topic = "gm_" + publish_topic;

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

      const credentials = await getCognitoCredentials();
      
      const deviceIot = awsIot.device({
          region: region,
          clientId: "gm" + iotclientId,
          accessKeyId: credentials.accessKey,
          secretKey: credentials.secretKey,
          sessionToken : credentials.sessionToken, 
          protocol: 'wss',
          port: 443,
          host: iotendpoint
      });

      deviceIot.on('message', function(_topic, payload) {
          var json = JSON.parse(payload.toString());
          command = json["command"];
          if (command == "update") {
            vue.LAPTIME = json["time"].toFixed(2);;
            vue.STATUS = json["status"];
            vue.DISTANCE = json["distance"].toFixed(2);

          }
      });
      
      deviceIot.subscribe(gm_subscribe_topic, undefined, function (err, granted){
          if( err ){
              console.log('subscribe error: ' + err);
          } else {
              console.log('subscribe success');
          }
      });

      let vue = new Vue({
        el: '#app',
        data: {
          LAPTIME:"-",
          STATUS:"-",
          DISTANCE:"-"
        },        
      methods: {
        start_game: function () {
          let payload = {};
          console.log("Game start!"); 
          request_id =  (new Date()).getTime();
          payload["command"] = "game";
          payload["action"] = "start";
          payload["request_id"] = request_id
          deviceIot.publish(gm_publish_topic, JSON.stringify(payload));
        }
      }
    })

    }

  main();
