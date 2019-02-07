var jsonData = null;
$(document).ready(function() {
  // EXTRACT JSON DATA.
  $.getJSON("config/config.json", function(data) {
    jsonData = data;
    $.each(jsonData.pinconfigs, function(index, value) {
      // APPEND OR INSERT DATA TO SELECT ELEMENT.
      $('#pinInput').append('<option value="' + value.pin + '">Pin #' + value.pin + '</option>');
    });
    setFormIdx(0);
  });

  //SELECT change EVENT TO READ SELECTED VALUE FROM DROPDOWN LIST.
  $('#pinInput').change(function() {
    setFormIdx(this.selectedIndex)
  });

  $("#submit").on('click', function() {

    formToJson();
    // send ajax
    var toSend = JSON.stringify(jsonData);
    $.ajax({
      url: '/index.html', // url where to submit the request
      type: "POST", // type of action POST || GET
      dataType: 'json', // data type
      data: toSend,
      contentType: "application/json; charset=UTF-8",
      success: function(result) {
        // you can see the result from the console
        // tab of the developer tools
        console.log("sent this");
        console.log(jsonData);
        console.log("done");
      },
      error: function(xhr, resp, text) {
        console.log(xhr, resp, text);
      }
    })
  });
});

function setFormIdx(idx) {
  if (jsonData == null) {
    return;
  }
  //General handling
  document.getElementById('addressInput').value = jsonData.pinconfigs[idx].address;

  if ((jsonData.pinconfigs[idx].type == "Input") || (jsonData.pinconfigs[idx].type == "input")) {
    document.getElementById('pin-Input').checked = true;
    showInputOuput(1);
  } else {
    document.getElementById('pin-Output').checked = true;
    showInputOuput(0);
  }
  if (jsonData.pinconfigs[idx].contact == 1) {
    document.getElementById('contactType1').checked = true;
  } else {
    document.getElementById('contactType2').checked = true;
  }
  //Input handling
  document.getElementById('ipActiveHigh').checked = (jsonData.pinconfigs[idx].activeHigh == 1);
  document.getElementById('ipSWODelay').checked = (jsonData.pinconfigs[idx].switchOffDelay == 1);
  document.getElementById('ipTurnout').checked = (jsonData.pinconfigs[idx].turnout == 1);
  document.getElementById('ipPulse').checked = (jsonData.pinconfigs[idx].inputPulse == 1);
  switchOffIP();
  pulseCfgIP();
  document.getElementById('pulseLenIP').value = jsonData.pinconfigs[idx].inputPulseLen;
  document.getElementById('switchOffIPtx').value = jsonData.pinconfigs[idx].switchOffDelayLen;

  //output handling
  document.getElementById('opLowOnStart').checked = (jsonData.pinconfigs[idx].lowOnStart == 1);
  document.getElementById('opPulse').checked = (jsonData.pinconfigs[idx].outputPulse == 1);
  document.getElementById('opFlash').checked = (jsonData.pinconfigs[idx].flash == 1);
  document.getElementById('opBlock').checked = (jsonData.pinconfigs[idx].blockDetctor == 1);
  flashCfgOP();
  pulseCfgOP();
  document.getElementById('pulseLenOP').value = jsonData.pinconfigs[idx].outputPulseLen;
  document.getElementById('flashOnOP').value = jsonData.pinconfigs[idx].flashOnLen;
  //  document.getElementById('flashOffOP').value = jsonData.pinconfigs[idx].flashOffLen;
}

function showInputOuput(cfg) {
  if (cfg === 1) {
    document.getElementById('contentOP').style.display = 'none';
    document.getElementById('OPConfig').style.display = 'none';
    document.getElementById('contentIP').style.display = 'block';
    document.getElementById('IPConfig').style.display = 'block';


  } else {
    document.getElementById('contentOP').style.display = 'block';
    document.getElementById('OPConfig').style.display = 'block';
    document.getElementById('contentIP').style.display = 'none';
    document.getElementById('IPConfig').style.display = 'none';
  }
}

function pulseCfgOP() {
  var lfckv = document.getElementById('opPulse').checked;
  if (lfckv) {
    document.getElementById('pulseConfigOP').style.display = 'block';
  } else {
    document.getElementById('pulseConfigOP').style.display = 'none';
  }
}

function switchOffIP() {
  var lfckv = document.getElementById('ipSWODelay').checked;
  if (lfckv) {
    document.getElementById('switchOffIP').style.display = 'block';
  } else {
    document.getElementById('switchOffIP').style.display = 'none';
  }
}

function pulseCfgIP() {
  var lfckv = document.getElementById('ipPulse').checked;
  if (lfckv) {
    document.getElementById('pulseConfigIP').style.display = 'block';
  } else {
    document.getElementById('pulseConfigIP').style.display = 'none';
  }
}

function flashCfgOP() {
  var lfckv = document.getElementById('opFlash').checked;
  if (lfckv) {
    document.getElementById('flashConfigOP').style.display = 'block';
  } else {
    document.getElementById('flashConfigOP').style.display = 'none';
  }
}

function hoverhelp(helpType) {
  switch (helpType) {
    case 1:
      document.getElementById('helpTitle').innerHTML = "Low On Startup";
      document.getElementById('help').innerHTML = "Starts with the output in the low state";
      break;
    case 2:
      document.getElementById('helpTitle').innerHTML = "Pulse Contact";
      document.getElementById('help').innerHTML = "Turns the ouput on for a configurable amount of time, and then turns it off again.";
      break;
    case 3:
      document.getElementById('helpTitle').innerHTML = "Flash";
      document.getElementById('help').innerHTML = "NOT YET SUPPORTED - Flashes the output on an off for a configurable amount of time";
      break;
    case 4:
      document.getElementById('helpTitle').innerHTML = "Block Detector";
      document.getElementById('help').innerHTML = "If checked this turns the ouput on whilst the switch request reports on. If it is not checked the direction (thrown/closed) is instead used to drive the output.";
      break;
    case 101:
      document.getElementById('helpTitle').innerHTML = "Active High";
      document.getElementById('help').innerHTML = "If this is ticked, then the input is assumed to be on when a high logic level is present.";
      break;
    case 102:
      document.getElementById('helpTitle').innerHTML = "Switch Off Delay";
      document.getElementById('help').innerHTML = "Any transitions from on to of are only reported after the specified delay";
      break;
    case 103:
      document.getElementById('helpTitle').innerHTML = "Turnout Sensor";
      document.getElementById('help').innerHTML = "Sets the input as a turnout sensor.";
      break;
    case 104:
      document.getElementById('helpTitle').innerHTML = "Pulse";
      document.getElementById('help').innerHTML = "Inputs are only reported as on for the specifed amount of time. This is useful for having an input simulate a push to make button for controlling solenoids.";
      break;
    default:

  }

  document.getElementById('helpDiv').style.display = 'block';
}

function hideHelp() {
  document.getElementById('helpDiv').style.display = 'none';
}


function formToJson() {

  var idx = 0;


  idx = document.getElementById('pinInput').value - 1;


  jsonData.pinconfigs[idx].address = document.getElementById('addressInput').value

  if (document.getElementById('pin-Input').checked == true) {
    jsonData.pinconfigs[idx].type == "Input";
  } else {
    jsonData.pinconfigs[idx].type == "Output";
  }
  if (document.getElementById('contactType1').checked == true) {
    jsonData.pinconfigs[idx].contact == 1;
  } else {
    jsonData.pinconfigs[idx].contact == 2;
  }


  if (document.getElementById('ipActiveHigh').checked == true) {
    jsonData.pinconfigs[idx].activeHigh = 1;
  } else {
    jsonData.pinconfigs[idx].activeHigh = 0;
  }
  if (document.getElementById('ipSWODelay').checked == true) {
    jsonData.pinconfigs[idx].switchOffDelay = 1;
  } else {
    jsonData.pinconfigs[idx].switchOffDelay = 0;
  }
  if (document.getElementById('ipTurnout').checked == true) {
    jsonData.pinconfigs[idx].turnout = 1;
  } else {
    jsonData.pinconfigs[idx].turnout = 0;
  }
  if (document.getElementById('ipPulse').checked == true) {
    jsonData.pinconfigs[idx].inputPulse = 1;
  } else {
    jsonData.pinconfigs[idx].inputPulse = 0;
  }

  jsonData.pinconfigs[idx].inputPulseLen = document.getElementById('pulseLenIP').value;
  jsonData.pinconfigs[idx].switchOffDelayLen = document.getElementById('switchOffIPtx').value;



  if (document.getElementById('opLowOnStart').checked == true) {
    jsonData.pinconfigs[idx].lowOnStart = 1;
  } else {
    jsonData.pinconfigs[idx].lowOnStart = 0;
  }
  if (document.getElementById('opPulse').checked == true) {
    jsonData.pinconfigs[idx].outputPulse = 1;
  } else {
    jsonData.pinconfigs[idx].outputPulse = 0;
  }
  if (document.getElementById('opFlash').checked == true) {
    jsonData.pinconfigs[idx].flash = 1;
  } else {
    jsonData.pinconfigs[idx].flash = 0;
  }
  if (document.getElementById('opBlock').checked == true) {
    jsonData.pinconfigs[idx].blockDetctor = 1;
  } else {
    jsonData.pinconfigs[idx].blockDetctor = 0;
  }

  jsonData.pinconfigs[idx].outputPulseLen = document.getElementById('pulseLenOP').value;
  jsonData.pinconfigs[idx].flashOnLen = document.getElementById('flashOnOP').value;

};