import { NT4_Client } from "./NT4.js";

var topics = {}
var connected = false;

const nt4Client = new NT4_Client(
    window.location.hostname,
    "REBELLION-Dashboard",
    addedTopic => {
        // on topic announce
        console.log(`Topic announced: ${addedTopic}`);
    },
    removedTopic => {
        // on topic unannounce
        console.log(`Topic unannounced: ${removedTopic}`);
    },
    (topic, timestamp_us, value) => {
        // on topic data
        // console.log(`Topic data received: ${topic} at ${timestamp_us} with value:`, value);
        topics[topic.name] = {
            topic: topic,
            timestamp: timestamp_us,
            value: value
        };
        console.log(JSON.stringify(topic.properties))
        // TODO update UI with the received data
    },
    () => {
        // on connection
        console.log("Connected to NT4 server");
        // TODO display connected state in UI
    },
    () => {
        // on disconnect
        console.log("Disconnected from NT4 server");
        // TODO display disconnected state in UI
    }
);

window.addEventListener("load", () => {
    // Initialize the NT4 client when the window loads
    nt4Client.subscribe(
        [
            // topic to subscribe to
            "" // subscribe to all topics
        ],
        true, // prefix mode - catch all data below the given prefixes
    )

    // connect to websocket
    nt4Client.connect();
})

setInterval(() => {
    var timestamp = topics["/AdvantageKit/Timestamp"].value || 0;
    var displayString = ""
    Object.values(topics).forEach(topic => {
        displayString += `<b class="progress" style="--progress: ${Math.min((timestamp - topic.timestamp) / 100000, 99)}">${topic.topic.name}</b>: <code>${topic.value}</code> (${topic.topic.type})<br>`;
    })

    document.body.innerHTML = `${connected ? "Connected to NT4 server" : "Disconnected from NT4 server"}<br><br>${displayString}`;
}, 10); // Update every second
