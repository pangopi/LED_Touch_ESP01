// Constants
const states = ["Off", "On", "Changing", "Delay Off"]; 

// Variables
// [status, intensity]
let salon = [0 , 0];
let companionway = [0, 0];

// Create a client instance: Broker, Port, Websocket Path, Client ID
client = new Paho.MQTT.Client("192.168.5.1", Number(9001), "/ws", "Websocket" + Math.floor(Math.random() * 1001));

// set callback handlers
client.onConnectionLost = function (responseObject) {
    console.log("Connection Lost: "+responseObject.errorMessage);
    connect();
}

client.onMessageArrived = function (message) {
    console.log("Message Arrived - Topic: " + message.destinationName + " Payload: " + message.payloadString);
    // console.log("Topic:     " + message.destinationName);
    // console.log("QoS:       " + message.qos);
    // console.log("Retained:  " + message.retained);

    if (message.destinationName == "salon/light/main") {
        var status = JSON.parse(message.payloadString);
        if (status.state >= 0) {
            document.getElementById('stateSalon').innerHTML = states[status.state];
            salon[0] = status.state;
        }
        if (status.intensity >= 0) {
            document.getElementById('intensitySalon').innerHTML = status.intensity;
            document.getElementById('sliderSalon').value = status.intensity;
            salon[1] = status.intensity;
        }
    }

    if (message.destinationName == "companionway/light/main") {
        var status = JSON.parse(message.payloadString);
        if (status.state >= 0) {
            document.getElementById('stateCompanionway').innerHTML = states[status.state];
            companionway[0] = status.state;
        }
        if (status.intensity >= 0) {
            document.getElementById('intensityCompanionway').innerHTML = status.intensity;
            document.getElementById('sliderCompanionway').value = status.intensity;
            companionway[1] = status.intensity;
        }
    }
}

// Called when the connection is made
function onConnect(){
    console.log("Connected!");

    // Send message
    var message = new Paho.MQTT.Message("Websocket connected");
    message.destinationName = "housekeeping";
    message.qos = 0; // Optional, default 0
    client.send(message);

    client.subscribe("+/light/main");
    client.subscribe("+/light/main/command");
    client.subscribe("housekeeping");
    
    // Request current light status for all lights
    let payload = new Paho.MQTT.Message("{\"info\":1}");
    payload.destinationName = "salon/light/main/command";
    message.qos = 0;
    client.send(payload);

    payload.destinationName = "companionway/light/main/command";
    client.send(payload);
}

// Connect the client, with a Username and Password
function connect(){
    client.connect({
        onSuccess: onConnect, 
        userName : "pangolin",
        password : "mikeharris"
    });
}

function changeIntensity(lightid, amount){
    // Change the intensity with an arbitrary amount
    if (lightid == "salon") {
        if (salon[1] == 0 && amount > 0) {
            publish(`salon/light/main/command`, `{"state":2,"intensity":64}`);
        } else {
            publish(`salon/light/main/command`, `{"state":2,"intensity":${salon[1]+amount}}`);
        }
    }
    if (lightid == "companionway") {
        if (companionway[1] == 0 && amount > 0) {
            publish(`companionway/light/main/command`, `{"state":2,"intensity":64}`);
        } else {
            publish(`companionway/light/main/command`, `{"state":2,"intensity":${companionway[1]+amount}}`);
        }
    }

}

function publish(topic, payload){
    // Publish to MQTT
    var message = new Paho.MQTT.Message(payload);
    message.destinationName = topic;
    message.qos = 0;
    client.send(message);
};

connect();
