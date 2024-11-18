static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>

<head>
    <title>ESP32-CAMERA Teachable Machine</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <link rel="stylesheet" href="https:\/\/cdn.jsdelivr.net/npm/bootstrap@4.0.0/dist/css/bootstrap.min.css"
        integrity="sha384-Gn5384xqQ1aoWXA+058RXPxPg6fy4IWvTNh0E263XmFcJlSAwiGgFAW/dAiS6JXm" crossorigin="anonymous">
    <style>
        body {
            font-family: Arial, Helvetica, sans-serif;
            background: #181818;
            color: #EFEFEF;
            font-size: 16px;
            display: none;
        }
        h2 {
            font-size: 18px
        }
        section.main {
            display: flex
        }
        #menu,
        section.main {
            flex-direction: column
        }
        #menu {
            display: flex;
            flex-wrap: nowrap;
            min-width: 450px;
            background: #363636;
            padding: 8px;
            border-radius: 4px;
            margin-top: -10px;
            margin-right: 10px
        }
        #content {
            display: flex;
            flex-wrap: wrap;
            align-items: stretch
        }
        figure {
            padding: 0;
            margin: 0;
            -webkit-margin-before: 0;
            margin-block-start: 0;
            -webkit-margin-after: 0;
            margin-block-end: 0;
            -webkit-margin-start: 0;
            margin-inline-start: 0;
            -webkit-margin-end: 0;
            margin-inline-end: 0
        }
        figure img {
            display: block;
            width: 100%;
            height: auto;
            border-radius: 4px;
            margin-top: 8px
        }
        @media (min-width: 800px) and (orientation:landscape) {
            #content {
                display: flex;
                flex-wrap: nowrap;
                align-items: stretch
            }
            figure img {
                display: block;
                max-width: 100%;
                max-height: calc(100vh - 40px);
                width: auto;
                height: auto
            }
            figure {
                padding: 0;
                margin: 0;
                -webkit-margin-before: 0;
                margin-block-start: 0;
                -webkit-margin-after: 0;
                margin-block-end: 0;
                -webkit-margin-start: 0;
                margin-inline-start: 0;
                -webkit-margin-end: 0;
                margin-inline-end: 0
            }
        }
        section#buttons {
            display: flex;
            flex-wrap: nowrap;
            justify-content: space-between
        }
        #nav-toggle {
            cursor: pointer;
            display: block
        }
        #nav-toggle-cb {
            outline: 0;
            opacity: 0;
            width: 0;
            height: 0
        }
        #nav-toggle-cb:checked+#menu {
            display: none
        }
        .input-group {
            display: flex;
            flex-wrap: nowrap;
            line-height: 22px;
            margin: 5px 0
        }
        .input-group>label {
            display: inline-block;
            padding-right: 10px;
            min-width: 20%
        }

        .input-group input,
        .input-group select {
            flex-grow: 1
        }
        .range-max,
        .range-min {
            display: inline-block;
            padding: 0 5px
        }
        button {
            display: block;
            margin: 5px;
            padding: 0 12px;
            border: 0;
            line-height: 28px;
            cursor: pointer;
            color: #fff;
            background: #4135a8;
            border-radius: 5px;
            font-size: 16px;
            outline: 0
        }
        button:hover {
            background: #776ff1
        }
        button:active {
            background: #1100ff
        }
        button.disabled {
            cursor: default;
            background: #a0a0a0
        }
        input[type=range] {
            -webkit-appearance: none;
            width: 100%;
            height: 22px;
            background: #363636;
            cursor: pointer;
            margin: 0
        }
        input[type=range]:focus {
            outline: 0
        }
        input[type=range]::-webkit-slider-runnable-track {
            width: 100%;
            height: 2px;
            cursor: pointer;
            background: #EFEFEF;
            border-radius: 0;
            border: 0 solid #EFEFEF
        }
        input[type=range]::-webkit-slider-thumb {
            border: 1px solid rgba(0, 0, 30, 0);
            height: 22px;
            width: 22px;
            border-radius: 50px;
            background: #2c1497;
            cursor: pointer;
            -webkit-appearance: none;
            margin-top: -11.5px
        }
        input[type=range]:focus::-webkit-slider-runnable-track {
            background: #EFEFEF
        }
        input[type=range]::-moz-range-track {
            width: 100%;
            height: 2px;
            cursor: pointer;
            background: #EFEFEF;
            border-radius: 0;
            border: 0 solid #EFEFEF
        }
        input[type=range]::-moz-range-thumb {
            border: 1px solid rgba(0, 0, 30, 0);
            height: 22px;
            width: 22px;
            border-radius: 50px;
            background: #2c1497;
            cursor: pointer
        }
        input[type=range]::-ms-track {
            width: 100%;
            height: 2px;
            cursor: pointer;
            background: 0 0;
            border-color: transparent;
            color: transparent
        }
        input[type=range]::-ms-fill-lower {
            background: #EFEFEF;
            border: 0 solid #EFEFEF;
            border-radius: 0
        }
        input[type=range]::-ms-fill-upper {
            background: #EFEFEF;
            border: 0 solid #EFEFEF;
            border-radius: 0
        }
        input[type=range]::-ms-thumb {
            border: 1px solid rgba(0, 0, 30, 0);
            height: 22px;
            width: 22px;
            border-radius: 50px;
            background: #2c1497;
            cursor: pointer;
            height: 2px
        }
        input[type=range]:focus::-ms-fill-lower {
            background: #EFEFEF
        }
        input[type=range]:focus::-ms-fill-upper {
            background: #363636
        }
        .switch {
            display: block;
            position: relative;
            line-height: 22px;
            font-size: 16px;
            height: 22px
        }
        .switch input {
            outline: 0;
            opacity: 0;
            width: 0;
            height: 0
        }
        .slider {
            width: 50px;
            height: 22px;
            border-radius: 22px;
            cursor: pointer;
            background-color: grey
        }
        .slider,
        .slider:before {
            display: inline-block;
            transition: .4s
        }
        .slider:before {
            position: relative;
            content: "";
            border-radius: 50%;
            height: 16px;
            width: 16px;
            left: 4px;
            top: 3px;
            background-color: #fff
        }
        input:checked+.slider {
            background-color: #2c1497
        }
        input:checked+.slider:before {
            -webkit-transform: translateX(26px);
            transform: translateX(26px)
        }
        select {
            border: 1px solid #363636;
            font-size: 14px;
            height: 22px;
            outline: 0;
            border-radius: 5px
        }
        .image-container {
            position: relative;
            min-width: 160px
        }
        .close {
            position: absolute;
            right: 5px;
            top: 5px;
            background: #2c1497;
            width: 16px;
            height: 16px;
            border-radius: 100px;
            color: #fff;
            text-align: center;
            line-height: 18px;
            cursor: pointer
        }
        .hidden {
            display: none
        }
    </style>
    <script src="https:\/\/ajax.googleapis.com/ajax/libs/jquery/1.8.0/jquery.min.js"></script>
    <script src="https:\/\/cdn.jsdelivr.net/npm/@tensorflow/tfjs@1.3.1/dist/tf.min.js"></script>
    <script
        src="https:\/\/cdn.jsdelivr.net/npm/@teachablemachine/image@0.8/dist/teachablemachine-image.min.js"></script>
    <script src="https:\/\/cdn.jsdelivr.net/npm/@teachablemachine/pose@0.8/dist/teachablemachine-pose.min.js"></script>
</head>

<body>
    <div id="container">

        <div class="row" style="margin-top: 50px;">
            <div class="col-2">
            </div>
            <div class="col-5">

                <div id="logo">
                    <label for="nav-toggle-cb" id="nav-toggle">&#9776;&nbsp;&nbsp;Menu</label>
                </div>
                <div id="content">
                    <div id="sidebar">
                        <input type="checkbox" id="nav-toggle-cb">
                        <nav id="menu">
                            <div class="input-group">
                                <label for="kind">Model kind:</label>
                                <select id="kind">
                                    <option value="image">image</option>
                                    <option value="pose">pose</option>
                                </select>
                            </div>
                            <div class="input-group" style="display:none;">
                                <label for="modelPath">Model path:</label>
                                <input type="text" id="modelPath"
                                    value="https://teachablemachine.withgoogle.com/models/9vNwq0Zs9">  
                            </div>
                            <div class="input-group" style="display:none;">
                                <label for="btnModel"></label>
                                <button type="button" id="btnModel" onclick="LoadModel();">Start Recognition</button>
                            </div>
                            <div class="input-group" id="mirrorimage-group">
                                <label for="mirrorimage">Resolution</label>
                                <select id="mirrorimage" class="default-action">
                                    <option value="1">yes</option>
                                    <option value="0">no</option>
                                </select>
                            </div>
                            <div class="input-group" id="flash-group">
                                <label for="flash">Flash</label>
                                <div class="range-min">0</div>
                                <input type="range" id="flash" min="0" max="255" value="0" class="default-action">
                                <div class="range-max">255</div>
                            </div>
                            <div class="input-group" id="framesize-group">
                                <label for="framesize">Resolution</label>
                                <select id="framesize" class="default-action">
                                    <option value="10">UXGA(1600x1200)</option>
                                    <option value="9">SXGA(1280x1024)</option>
                                    <option value="8">XGA(1024x768)</option>
                                    <option value="7">SVGA(800x600)</option>
                                    <option value="6">VGA(640x480)</option>
                                    <option value="5" selected="selected">CIF(400x296)</option>
                                    <option value="4">QVGA(320x240)</option>
                                    <option value="3">HQVGA(240x176)</option>
                                    <option value="0">QQVGA(160x120)</option>
                                </select>
                            </div>
                            <div class="input-group" id="quality-group">
                                <label for="quality">Quality</label>
                                <div class="range-min">10</div>
                                <input type="range" id="quality" min="10" max="63" value="10" class="default-action">
                                <div class="range-max">63</div>
                            </div>
                            <div class="input-group" id="brightness-group">
                                <label for="brightness">Brightness</label>
                                <div class="range-min">-2</div>
                                <input type="range" id="brightness" min="-2" max="2" value="0" class="default-action">
                                <div class="range-max">2</div>
                            </div>
                            <div class="input-group" id="contrast-group">
                                <label for="contrast">Contrast</label>
                                <div class="range-min">-2</div>
                                <input type="range" id="contrast" min="-2" max="2" value="0" class="default-action">
                                <div class="range-max">2</div>
                            </div>
                            <div class="input-group" id="rotate-group">
                                <label for="rotate">Rotate:</label>
                                <select
                                    onchange="document.getElementById('canvas').style.transform='rotate('+this.value+')';">
                                    <option value="0deg">0deg</option>
                                    <option value="90deg">90deg</option>
                                    <option value="180deg">180deg</option>
                                    <option value="270deg">270deg</option>
                                </select>
                            </div>
                        </nav>
                    </div>
                </div>
                <div id="result" style="color:red">
                    <div>
                    </div>
                </div>
            </div>
            <div class="col-5" style="margin-top: 50px;">
                <img id="ShowImage" src="" style="display:none">
                <canvas id="canvas" style="display: none;"></canvas>

                <section id="buttons">
                    <table>
                        <tr>
                            <td><button type="button" id="restart">Restart</button></td>
                            <td colspan="2"><button type="button" id="getStill" style="display:none">Get Still</button>
                            </td>
                        </tr>
                    </table>
                </section>
            </div>
</body>

</html>

<script>
    var getStill = document.getElementById('getStill');
    var ShowImage = document.getElementById('ShowImage');
    var canvas = document.getElementById("canvas");
    var context = canvas.getContext("2d");
    var mirrorimage = document.getElementById("mirrorimage");
    var flash = document.getElementById('flash');
    var modelPath = document.getElementById('modelPath');
    var result = document.getElementById('result');
    var kind = document.getElementById('kind');
    var myTimer;
    var restartCount = 0;

    async function LoadModel() {
        if (modelPath.value == "") {
            result.innerHTML = "Please input model path.";
            return;
        }

        result.innerHTML = "Please wait for loading model.";

        const URL = modelPath.value;
        const modelURL = URL + "/model.json";
        const metadataURL = URL + "/metadata.json";

        if (kind.value == "image") {
            Model = await tmImage.load(modelURL, metadataURL);
        }
        else if (kind.value == "pose") {
            Model = await tmPose.load(modelURL, metadataURL);
        }
        maxPredictions = Model.getTotalClasses();
        result.innerHTML = "";

        getStill.style.display = "block";
        getStill.click();
        document.body.style.display = "block";
    }
    document.addEventListener('DOMContentLoaded', function() {
    LoadModel();
    });
    getStill.onclick = function (event) {
        clearInterval(myTimer);
        myTimer = setInterval(function () { error_handle(); }, 5000);
        ShowImage.src = location.origin + '/?getstill=' + Math.random();
    }

    function error_handle() {
        restartCount++;
        clearInterval(myTimer);
        if (restartCount <= 2) {
            //message.innerHTML = "Get still error. <br>Restart ESP32-CAM "+restartCount+" times.";
            myTimer = setInterval(function () { getStill.click(); }, 6000);
        }
        else
            message.innerHTML = "Get still error. <br>Please close the page and check ESP32-CAM.";
    }

    ShowImage.onload = function (event) {
        clearInterval(myTimer);
        restartCount = 0;
        canvas.setAttribute("width", ShowImage.width);
        canvas.setAttribute("height", ShowImage.height);
        canvas.style.display = "block";

        if (mirrorimage.value == 1) {
            context.translate((canvas.width + ShowImage.width) / 2, 0);
            context.scale(-1, 1);
            context.drawImage(ShowImage, 0, 0, ShowImage.width, ShowImage.height);
            context.setTransform(1, 0, 0, 1, 0, 0);
        }
        else
            context.drawImage(ShowImage, 0, 0, ShowImage.width, ShowImage.height);
        predict();
    }

    restart.onclick = function (event) {
        fetch(location.origin + '/?restart=stop');
    }
    framesize.onclick = function (event) {
        fetch(document.location.origin + '/?framesize=' + this.value + ';stop');
    }
    flash.onchange = function (event) {
        fetch(location.origin + '/?flash=' + this.value + ';stop');
    }
    quality.onclick = function (event) {
        fetch(document.location.origin + '/?quality=' + this.value + ';stop');
    }
    brightness.onclick = function (event) {
        fetch(document.location.origin + '/?brightness=' + this.value + ';stop');
    }
    contrast.onclick = function (event) {
        fetch(document.location.origin + '/?contrast=' + this.value + ';stop');
    }

    async function predict() {
        var data = "";
        var maxClassName = "";
        var maxProbability = "";

        canvas.setAttribute("width", ShowImage.width);
        canvas.setAttribute("height", ShowImage.height);
        context.drawImage(ShowImage, 0, 0, ShowImage.width, ShowImage.height);

        if (kind.value == "image")
            var prediction = await Model.predict(canvas);
        else if (kind.value == "pose") {
            var { pose, posenetOutput } = await Model.estimatePose(canvas);
            var prediction = await Model.predict(posenetOutput);
        }

        if (maxPredictions > 0) {
            for (let i = 0; i < maxPredictions; i++) {
                if (i == 0) {
                    maxClassName = prediction[i].className;
                    maxProbability = prediction[i].probability;
                }
                else {
                    if (prediction[i].probability > maxProbability) {
                        maxClassName = prediction[i].className;
                        maxProbability = prediction[i].probability;
                    }
                }
                data += prediction[i].className + "," + prediction[i].probability.toFixed(2) + "<br>";
            }
            result.innerHTML = data;
            result.innerHTML += "<br>Result: " + maxClassName + "," + maxProbability;

            $.ajax({ url: document.location.origin + '/?serial=' + maxClassName + ";" + maxProbability + ';stop', async: false });
        }
        else
            result.innerHTML = "Unrecognizable";

        getStill.click();
    }   
</script>
)rawliteral";
