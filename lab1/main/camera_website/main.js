import { Elemental } from "https://deno.land/x/elementalist@0.5.35/main/deno.js?code"  // this makes writing html easy
import { showToast, showErrorToast } from "https://deno.land/x/good_component@0.2.14/main/actions/show_toast.js" // helpful pop-up tools (google "toast notifiction")
import { playSound } from "https://deno.land/x/animalese@1.0.1.0/animalese.js"
import { Face } from "./helpers/face.js" // this is a more complicated custom element that has animated eyes/expressions
import { fadeAfterNoInteraction } from "./helpers/opacity_helper.js" // this is a timing-helper

// 
// 
// Summary
// 
// 
    // 1. Let me quickstart you 
    //    A. run the `run/1_camera_server` and open up the URL
    //       on a desktop/laptop browser
    //    B. Open up the console of that browser
    //       (tutorial here: https://appuals.com/open-browser-console/ )
    //    C. Type `face.actions.showHappy()` and press enter
    //       then type `face.actions.relax()` and press enter
    //    D. You should see some stuff on the screen!
    //       Note there are other actions like .showConfusion() and .showScared()
    // 
    // 2. Tools!
    // 
    //    Three tools you should know about
    //    A. showToast(`Howdy`) is your friend. It makes a little pop up with the message
    //    B. logMessage(`Howdy`) is also your friend, because console.log() doesn't 
    //       work on a phone but logMessage does
    //    C. You can play with any variable in the browser console if you do: 
    //          window.theVariable = theVariable
    //          // examples are at the very bottom
    //      It helps a lot for debugging on a desktop/laptop
    //     
    // 3. You need to edit the "Events" section towards the bottom
    // 
    //    The section is a list of functions
    //    Each function will be called when a different event happens
    //    (its not magic either, you can find the code that calls those functions in this file)
    //    
    //    This is where you can trigger stuff like `face.action.showHappy()`
    // 
    // 4. If you want to add your own HTML elements to the page
    //    
    //    A. There's a weird thing I need to tell you about first.
    //       Normally we would edit the index.html. However,
    //       thats not the case for this project, because there is code
    //       in this file that replaces the <body> tag.
    //       So if you want to add html elements, you'll need to 
    //       edit the "Elements" section in this file.
    // 
    //    B. You can write html inside this file like so:
    //          let yourElement = html`
    //              <div>I'm html</div>
    //          `
    // 
    //    C. If you see some stack overflow / ChatGPT answers that look like this:
    //           <div>
    //              <input id="my-input" placeholder="howdy" >
    //           </div>
    //           
    //           let element = document.getElementById("my-input")
    //  
    //           // or: let element = document.querySelector("#my-input")
    //           // or let element = $("#my-input")
    // 
    //       THROW THE ANSWER IN THE TRASH
    //       Replace it with:
    // 
    //          let element = html`<input placeholder="howdy">`
    //          html`
    //              <div>
    //                  ${element}
    //              </div>
    //          `

// 
// Globals
// 
    const parameters = {
        frameSendRate: 2000, // 200 means it sends a frame every 200ms (5fps)
                            // NOTE: if this is too fast it can overwhelm the python code!
                            //       make number smaller if python is getting overloaded 
        audioBufferSize: 2048,
        defaultPort: 9093,
        videoWidth: 640,
        videoHeight: 420,
    }
    const rosTopics = {
        audioTopic: null,
        imageTopic: null,
    }

// 
// Custom Elements
// 
    function MessageLog({ ...props }) {
        // note all the <br>'s are to help with viewing on mobile 
        return MessageLog.element = html`
            <span
                style="padding: 1rem; position: fixed; right: 0; top: 0; height: 60vh; overflow: auto; width: 21rem; background-color: rgba(0,0,0,0.18); color: white; border-left: 2px gray solid; border-bottom: 2px gray solid; box-shadow: 0 4px 5px 0 rgba(0,0,0,0.14), 0 1px 10px 0 rgba(0,0,0,0.12), 0 2px 4px -1px rgba(0,0,0,0.3); z-index: 1;"
                >
                (message log)
            </span>
        `
    }
    MessageLog.logHtml = function (...messages) {
        if (MessageLog.element) {
            const message = messages.join(" ")
            MessageLog.element.innerHTML += `<br>...<br>${message}`
            MessageLog.element.scrollTop = MessageLog.element.scrollHeight
        }
    }
    MessageLog.logMessage = function (...messages) {
        if (MessageLog.element) {
            const message = messages.join(" ")
            const escapedText = new Option(message).innerHTML
            MessageLog.element.innerHTML += `<br>...<br>${escapedText.replace(/\n/g,"<br>")}`
            MessageLog.element.scrollTop = MessageLog.element.scrollHeight
        }
    }


    let height
    function CameraSwitch({ children, ...props }) {
        const switchInput = html`<input type="checkbox" value="">`
        const video = html`<video muted style="display: none" autoplay></video>`
        const canvas = html`<canvas style="display: none"></canvas>`
        
        // function that is run once scale the height of the video stream to match the configured target width
        let hasRunOnce = false
        video.addEventListener(
            "canplay",
            function (event) {
                if (!hasRunOnce) {
                    height = video.videoHeight / (video.videoWidth / parameters.videoWidth)
                    video.setAttribute("width", parameters.videoWidth)
                    video.setAttribute("height", height)
                    canvas.setAttribute("width", parameters.videoWidth)
                    canvas.setAttribute("height", height)
                    hasRunOnce = true
                }
            },
            false
        )
        
        let cameraTimer = null
        let cameraStream = null
        switchInput.addEventListener(
            "click",
            // whenever the switch was clicked, run this function
            async function (event) {
                if (cameraTimer == null) {
                    // ros.connect("ws://" + window.location.hostname + ":9090");
                    RosConnecter.setupRosIfNeeded()
                    
                    if (!navigator.mediaDevices) {
                        MessageLog.logMessage(`Error: check the URL\nMake sure it has "https" and not "http"`)
                    } else {
                        try {
                            cameraStream = await navigator.mediaDevices.getUserMedia({
                                video: true,
                                audio: true,
                            })
                            video.srcObject = cameraStream
                            video.play()
                            // whenever the media is loaded, run this function
                            video.onloadedmetadata = function (event) {
                                height = video.videoHeight / (video.videoWidth / parameters.videoWidth)
                                video.setAttribute("width", parameters.videoWidth)
                                video.setAttribute("height", height)
                                canvas.setAttribute("width", parameters.videoWidth)
                                canvas.setAttribute("height", height)
                            }
                        } catch (error) {
                            MessageLog.logMessage(`Looks like there was an issue connecting to the camera. Make sure this browser can actually connect to your camera (for example try logging into Zoom and using "Your Room" and try turning on the camera)`)
                            throw error
                        }

                        try {
                            const audioCtx = new (window.AudioContext || window.webkitAudioContext)()
                            const source   = audioCtx.createMediaStreamSource(cameraStream)
                            const recorder = audioCtx.createScriptProcessor(parameters.audioBufferSize, 1, 1)
                            
                            // whenever the switch was clicked, run this function
                            recorder.onaudioprocess = function (event) {
                                rosTopics.audioTopic.publish(
                                    new ROSLIB.Message({
                                        data: Array.from(
                                            new Float32Array(
                                                event.inputBuffer.getChannelData(0)
                                            )
                                        ),
                                    })
                                )
                            }
                            
                            source.connect(recorder)
                            recorder.connect(audioCtx.destination)
                        } catch (error) {
                            MessageLog.logMessage(`Looks like there was an issue connecting to the microphone. Make sure this browser can actually connect to your camera (for example try logging into Zoom and using "Your Room" and try turning on the camera)`)
                            throw error
                        }
                    }
                    cameraTimer = setInterval(
                        // call takePicture at the frameSendRate
                        function(){
                            takePicture()
                        },
                        parameters.frameSendRate,
                    )
                } else {
                    ros.close()
                    cameraStream.stop()
                    hasRunOnce = false
                    takePicture() // blank the screen to prevent last image from staying
                    clearInterval(cameraTimer)
                    cameraTimer = null
                }
            },
            false
        )
        
        // function that is run by trigger several times a second
        // takes snapshot of video to canvas, encodes the images as base 64 and sends it to the ROS topic
        function takePicture() {
            if (!rosTopics.imageTopic) {
                if (RosConnecter.rosIsSetup) {
                    MessageLog.logMessage("Trying to take a picture but rosTopics.imageTopic is null")
                }
            } else {
                // MessageLog.logMessage("Trying to take a picture")
                canvas.width = parameters.videoWidth
                canvas.height = height

                canvas.getContext("2d").drawImage(video, 0, 0, canvas.width, canvas.height)

                var data = canvas.toDataURL("image/jpeg")
                var a = document.createElement("a")
                a.href = data
                var imageMessage = new ROSLIB.Message({
                    format: "jpeg",
                    data: data.replace("data:image/jpeg;base64,", ""),
                })

                rosTopics.imageTopic.publish(imageMessage)
            }
        }
        CameraSwitch.takePicture = takePicture
        
        return html`
            <div class="switch"
                style="margin-bottom: 2rem; width: 100%; display: flex; justify-content: space-between; align-items: center;">
                <h5>
                    Activate Camera
                </h5>
                ${video}
                <label>
                    ${switchInput}
                    <span class="lever"></span>
                </label>
            </div>
        `
    }
    // Singleton component
    function RosConnecter() {
        try {
            const ipAddressInput = html`<input type="text" placeholder="IP Address" value=${"" + window.location.hostname} color=white />`
            const portInput = html`<input type="text" placeholder="Port" value="${parameters.defaultPort}" color=white />`
            const connectButton = html`
                <button class="btn-large waves-effect waves-light" style="margin-top: 1rem; z-index: 999;">
                    Connect to ROSbridge Server
                </button>
            `
            connectButton.addEventListener("click", function(event) {
                RosConnecter.setupRosIfNeeded()
            })
            
            RosConnecter.setupRosIfNeeded = function () {
                if (!RosConnecter.rosIsSetup) {
                    // 
                    // create websocket URL
                    // 
                    let ipAddress = ipAddressInput.value
                    let port      = portInput.value
                    
                    // 
                    // check ipAddress
                    // 
                    if (ipAddress == "") {
                        // default to localhost if no ip address is provided
                        ipAddress = window.location.hostname
                        if (ipAddress == "localhost") {
                            ipAddress = "127.0.0.1"
                        }
                        if (ipAddress == "127.0.0.1") {
                            window.alert(`Note: your connection should be something other than localhost (aka 127.0.0.1).\n\nI'll still let you try with localhost but just note that the public facing address should be used instead.`)
                        }
                    }
                    
                    const baseValue = `${ipAddress}:${port}`
                    const url = `wss://${ipAddress}:${port}`
                    console.log(`Attempting to connect to: ${url}`)
                    
                    // 
                    // connect
                    // 
                    try {
                        const ros = new ROSLIB.Ros({
                            url: url,
                        })
                        
                        ros.on("connection", function () {
                            console.log("Connected to websocket server.")
                            RosConnecter.rosIsSetup = true
                            MessageLog.logMessage("Success!")
                        })

                        ros.on("error", function (error) {
                            console.log("Error connecting to websocket server: ", error)
                            MessageLog.logHtml(`1. Make sure <code>run/2_ros_bridge</code> is running<br>2. Try opening this in a new tab:<br><a href="https://${baseValue}">https://${baseValue}</a><br>3. Click Advanced -> Accept Risk and Continue<br>4.Then re-run this test<br>`)
                            showErrorToast(`Didn't Connect to socket\nSee log ->\n\n(Click to make this go away)`, {position: 'left',})
                        })

                        ros.on("close", function () {
                            RosConnecter.rosIsSetup = false
                            console.log("Connection to websocket server closed.")
                        })
                        
                        afterRosConnected(ros)
                    } catch (error) {
                        MessageLog.logMessage(`error connecting to ROS`)
                        console.error(`The error below is probably because of a url issue\nThe url given to ROS was: ${url}`)
                        console.error(error)
                    }
                    
                }
            }
            return html`
                <div>
                    <label for="ip">IP Address</label>
                    ${ipAddressInput}

                    <label for="port">Port</label>
                    ${portInput}

                    ${connectButton}
                </div>
            `
        } catch (error) {
            console.debug(`error is:`,error)
            throw error
        }
    }
// 
// Register custom elements
// 
    const html = Elemental({
        MessageLog,
        CameraSwitch,
        RosConnecter,
        Face,
    })
// 
// 
// Helpers
// 
// 
    // if you want to make helpers, this is a good place to put them
    
// 
// 
// Elements
// 
// 
    let face = html`<Face height=500 width=3000 style="position: fixed; top: 0rem; right: calc(50vw); transform: translateX(50%);" />`
    let controls = html`
        <div
            style="display: flex; position: fixed; bottom: 0rem; right: 0; flex-direction: column; width: 26rem; transform: scale(0.8) translate(17%, 18%); padding: 2rem; margin: 1rem; border-radius: 12px; background-color: rgba(0,0,0,0.18); color: white; transition: all 0.2s ease-in-out 0s; z-index: 2; border-radius: 0;"
            >
            <CameraSwitch></CameraSwitch>
            <RosConnecter></RosConnecter>
        </div>
    `
    document.body = html`
        <body style="background: #4b5e6b">
            ${face}
            
            <MessageLog></MessageLog>
            ${controls}
        </body>
    `

// 
// handle fading out the controls
// 
    const userInteractedWithPageFunc = fadeAfterNoInteraction({
        baseDelaySeconds: 5, 
        opacityLossPerSecond: 0.5,
        callback: function(newOpacity) {
            // reduce the opacity after a certain amount of no-interaction time
            // (newOpacity) will get smaller and smaller with each function call
            controls.style.opacity = newOpacity
            MessageLog.element.style.opacity = newOpacity
        },
    })
    document.body.addEventListener("mouseover", function (event) { userInteractedWithPageFunc() })
    document.body.addEventListener("mousemove", function (event) { userInteractedWithPageFunc() })
    document.body.addEventListener("click", function (event) { userInteractedWithPageFunc() })

// 
// 
// Events
// 
// 
    async function afterReceiveBackendMessage(data) {
        // 
        // EDIT ME
        // 
        console.debug(`data is:`,data)

        let showOnWebpage = false
        if (showOnWebpage) {
            MessageLog.logMessage(JSON.stringify(data))
        }
    }
    
    // the function below gets run after the UI button is pressed AND ros is actually able to connect
    async function afterRosConnected(ros) {
        // NOTE: you probably dont need to edit this function
        //       read it if you want to know how ros works

        // 
        // setup topics
        // 
        rosTopics.audioTopic = new ROSLIB.Topic({
            ros: ros,
            name: "/audio",
            messageType: "std_msgs/Float32MultiArray",
        })
        
        rosTopics.imageTopic = new ROSLIB.Topic({
            ros: ros,
            name: "/camera/image/compressed",
            messageType: "sensor_msgs/CompressedImage",
        })
        
        // 
        // listen for incoming data
        // 
        new ROSLIB.Topic({
            ros : ros,
            name : '/camera_server/do_something', // NOTE: this needs to be the same as the string in the python code
            messageType : 'std_msgs/String'
        }).subscribe((message) => {
            let data 
            try {
                data = JSON.parse(message.data)
            } catch (error) {
                MessageLog.logMessage(`error parsing message json`)
            }
            afterReceiveBackendMessage(data)
        })
    }

// 
// for debugging
// 
    // (global variables so you can play with them in the browser console)
    window.face           = face
    window.logMessage     = MessageLog.logMessage
    window.showToast      = showToast
    window.showErrorToast = showErrorToast
    window.ROSLIB         = ROSLIB