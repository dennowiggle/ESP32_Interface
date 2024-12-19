/*************************************************************************
 * Z80 Retro! ESP Programmer Web Interface
 * Code & ESP Programmer Board by Denno Wiggle
 * Z80 Retro! project by John Winans
 *  - https://github.com/Z80-Retro
 * 
 *************************************************************************/

/*************************************************************************
 * The web interface for this project was inspired by code written by 
 * user bitfixer for the romulator project, some parts of which have been 
 * reused.
 * https://github.com/bitfixer/bf-romulator
 * 
 *************************************************************************/
var updateTimeoutId;
var progArea;

// Placeholder for future
window.onload = function() {
    progArea = document.getElementById("progressAreaZ80");
}

function disableButtons()
{
    programButtonZ80.disabled = true;
    programButtonFpga.disabled = true;
    resetButtonZ80.disabled = true;
    resetButtonEsp.disabled = true;
    getBinButton.disabled = true;
    progressAreaZ80.innerHTML = "<br/>";
    progressAreaFpga.innerHTML = "<br/>";
    getBinResult.innerHTML = "<br/>";
    resetZ80Result.innerHTML = "<br/>";
    resetEspResult.innerHTML = "<br/>";
}

function enableButtons()
{
    programButtonZ80.disabled = false;
    programButtonFpga.disabled = false;
    resetButtonZ80.disabled = false;
    resetButtonEsp.disabled = false;
    getBinButton.disabled = false;
}

function uploadFile(isZ80File, fileId) 
{
    var inputTag = document.getElementById(fileId);
    fileSize = 0;
    if (!inputTag.files) 
    { 
        console.error("This browser doesn't seem to support the `files` property of file inputs.");
    } else if (!inputTag.files[0]) {
        alert("Please select a file before clicking Program button");
    } else {
        var file = inputTag.files[0];
        fileSize = file.size;
        // alert("File " + file.name + " is " + file.size + " bytes in size");
    }

    if (fileSize > 0)
    {
        disableButtons();
        let xhr = new XMLHttpRequest();
        if (isZ80File)
        {
            xhr.open("POST", "/uploadZ80?c="+fileSize);
        } else {
            xhr.open("POST", "/uploadFpga?c="+fileSize);
        }
        xhr.upload.addEventListener("progress", ({loaded, total}) =>{
            pct = Math.round(100*loaded/total);
            progArea.innerHTML = "uploading: " + pct.toString() + "%";
        });
        xhr.upload.addEventListener("loadend", event => { 
            progArea.innerHTML = "programming:";
            setTimeout(getProgress, 1000);
        });
        let formData = new FormData();
        formData.append("thefile", file);
        xhr.send(formData);
    }
}

function uploadFileZ80() 
{
    progArea = document.getElementById("progressAreaZ80");
    fileId = "flashFileZ80";
    uploadFile(true, fileId);
}

function uploadFileFpga() 
{
    progArea = document.getElementById("progressAreaFpga");
    fileId = "flashFileFpga";
    uploadFile(false, fileId);
}

function getProgress() {
    let progressRequest = new XMLHttpRequest();
    progressRequest.open("GET", "/progress");
    progressRequest.addEventListener("loadend", event => {
        if (progressRequest.responseText == "100")
        {
            progArea.innerHTML = "programming: done.";
            enableButtons();    
        }
        else
        {
            progArea.innerHTML = "programming: " + progressRequest.responseText + "%";
            setTimeout(getProgress, 1000);
        }
    });
    progressRequest.send();
}

function resetZ80()
{
    disableButtons();
    getRequestWithEndpoint("resetZ80", "", resetDoneZ80);
}

function resetEsp()
{
    disableButtons();
    getRequestWithEndpoint("resetEsp", "", resetDoneEsp);
}

function resetDoneZ80()
{
    enableButtons();
}

function resetDoneEsp()
{
    setTimeout(window.location.reload(), 5000);
}

function getBinImage() {
    window.location.href='/getBinImage';
}

function getRequestWithEndpoint(endpoint, args, done) {
    endpointName = "/" + endpoint;
    resultArea = document.getElementById(endpoint + "Result");
    req = new XMLHttpRequest();
    req.open("GET", endpointName + args);
    req.addEventListener("loadend", event => {
        resultArea.innerHTML = req.responseText;
        if (done != null) 
        {
            done();
        }
    });
    req.send();
}


function validateSizeZ80(input) {
    var fileSize = input.files[0].size;
    if (fileSize > 64 * 1024) {
        alert('File size exceeds 64kB');
        programButtonZ80.disabled = true;
    } else {
        programButtonZ80.disabled = false;
    }
}

function validateSizeFpga(input) {
    var fileSize = input.files[0].size;
    if (fileSize > 4 * 1024 * 1024) {
        alert('File size exceeds 4MB');
        programButtonFpga.disabled = true;
    } else {
        programButtonFpga.disabled = false;
    }
}