import { NT4_Client } from "./NT4.js";

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
        console.log(`Topic data received: ${topic} at ${timestamp_us} with value:`, value);
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
    nt4Client.subscribe([
        // topic to subscribe to
        "" // subscribe to all topics
    ],
        true, // prefix mode - catch all data below the given prefixes
        false, // send all (//TODO needs clarification)
        0.02 // period
    )

    // connect to websocket
    nt4Client.connect();
})
