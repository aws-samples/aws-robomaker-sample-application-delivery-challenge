
  const awsIot = require('aws-iot-device-sdk');

  var checked_team_email = "";

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
        //console.log(payload.toString())
        let json = JSON.parse(payload.toString());
        let command = json["command"];
        if (command == "update") {
          let distance = json["distance"].toFixed(2);
          let gameid = json["gameid"];
          if (gameid.length > 0) {
            gameid = "(game id:" + gameid + ")";
          }
          if (distance > 1000) {
            distance = "-";
          }
          vue.LAPTIME = json["time"].toFixed(2);;
          vue.STATUS = json["status"];
          vue.DISTANCE = distance;
          vue.isGameReady = json["evalid"];
          vue.GAMEID = gameid;

          if (vue.team_email.length == 0 && vue.isGameReady) {
            // page may be reloaded. Get email address from simulator
            let payload = {};
            let request_id =  (new Date()).getTime();
            payload["command"] = "game";
            payload["action"] = "get_team_email";
            payload["request_id"] = request_id;
            deviceIot.publish(gm_publish_topic, JSON.stringify(payload));    
          }
        } else if (command == "get_team_email_response") {
          vue.team_email = json["teamemail"];
        } else if (command == "show_message") {
          alert(json["message"]);
        }


    });
      
    deviceIot.subscribe(gm_subscribe_topic, undefined, function (err, granted){
        if( err ){
            console.log('subscribe error: ' + err);
        } else {
            console.log('subscribe success');
        }
    });

    var validate_team = function() {
        checked_team_email = this.team_email;

        console.log("Check team entry")

        vue.STATUS = "Checking..";
        vue.DISTANCE = "-";
        
        let payload = {};
        let request_id =  (new Date()).getTime();
        payload["command"] = "game";
        payload["action"] = "validate";
        payload["team_email"] = vue.team_email;
        payload["request_id"] = request_id;
        deviceIot.publish(gm_publish_topic, JSON.stringify(payload));
    }

    let vue = new Vue({
      el: '#app',
      data: {
        LAPTIME:"-",
        STATUS:"-",
        DISTANCE:"-",
        team_email:"",
        GAMEID:"",
        isGameReady : false
      },        
    methods: {
      start_game: function () {
        let payload = {};
        console.log("Game start!"); 
        let request_id =  (new Date()).getTime();
        payload["command"] = "game";
        payload["action"] = "start";
        payload["request_id"] = request_id
        deviceIot.publish(gm_publish_topic, JSON.stringify(payload));
      },
      team_email_focus_out: function() {
        if (this.team_email != checked_team_email) 
          validate_team();
      },
      team_email_enter: function() {
        validate_team();
      }
    }
  })

}

  main();
