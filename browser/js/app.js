
const awsIot = require('aws-iot-device-sdk');

var OdometryControlInstance;
var SyncJobFunction;
var SaveMapFunction;
var GoToTargetFunction;
var MoveAction = "";
var MoveActionPrev = "";
var SyncLastTime = 0;

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

  deviceIot.subscribe(subscribe_topic, undefined, function (err, granted){
      if( err ){
          console.log('subscribe error: ' + err);
      } else {
          console.log('subscribe success');
      }
  });

  deviceIot.on('message', function(_topic, payload) {
    var json = JSON.parse(payload.toString());
    let command = json["command"];
    if (command == "location") {

      let odom = json["odom"];
      if (odom) {
        OdometryControlInstance.setState({
          x:odom["x"].toFixed(4),
          y:odom["y"].toFixed(4),
          z:odom["z"].toFixed(4),
          h:(odom["yaw"] * ( 180 / Math.PI )).toFixed(4)
        })
      }
    }
    else if (command == "result") {
        alert(json["message"])
    }
  });

  //----
  
  SyncJobFunction = function() {
    let payload = {};
    let shouldPublish = false;
    if (MoveAction !== "") {
      if (MoveAction != MoveActionPrev) {
        MoveActionPrev = MoveAction;
        shouldPublish = true;
      }
      let t = (new Date()).getTime();
      if (t > SyncLastTime + 1500) {
        SyncLastTime = t;
        shouldPublish = true;
      } 
    }
    if (shouldPublish) {
      console.log("Sync:" + MoveAction);
      payload["command"] = "move";
      payload["action"] = MoveAction;
      shouldPublish = true;
      deviceIot.publish(publish_topic, JSON.stringify(payload));
    }
  }

  GoToTargetFunction = function(x,y, heading) {
    console.log("go to target!!");
    let payload = {};
    let request_id =  (new Date()).getTime();
    payload["command"] = "navigation";
    payload["action"] = "setGoal";
    payload["request_id"] = request_id
    payload["x"] = Number(x);
    payload["y"] = Number(y);
    payload["yaw"] = heading * ( Math.PI / 180 ) ;
    console.log(payload);
    deviceIot.publish(publish_topic, JSON.stringify(payload));
    MoveAction = "";
  };

  SaveMapFunction = function() {
    let payload = {};
    console.log("save map!!");
    let request_id =  (new Date()).getTime();
    payload["command"] = "map";
    payload["action"] = "save";
    payload["request_id"] = request_id
    console.log(payload);
    deviceIot.publish(publish_topic, JSON.stringify(payload));
  }


  setInterval(SyncJobFunction, 1000);  
}

main();




