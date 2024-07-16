var Socket;
var MotorSpeed = 0;
var StartRPMSliderValue = 0;

var TimeHour = 0;
var TimeMin = 0;
var TimeSec = 0;


let rpmValues = [];
let timerpmValues = [];
let RPMChart = new Chart(document.getElementById("RPM-CHART"), {
  type: "line",
  data: {
    labels: timerpmValues,
    datasets: [{
      fill: false,
      lineTension: 0,
      backgroundColor: "rgba(0,0,255,1.0)",
      borderColor: "rgba(0,0,255,0.1)",
      data: rpmValues
    }]
  },
  options: {
    legend: {display: false},
    scales: {
      yAxes: [{ticks: {min: 0, max:100}}],
    }
  }
});

let voltValues = [];
let timevoltValues = [];
let VoltChart = new Chart(document.getElementById("VOLT-CHART"), {
  type: "line",
  data: {
    labels: timevoltValues,
    datasets: [{
      fill: false,
      lineTension: 0,
      backgroundColor: "rgba(0,0,255,1.0)",
      borderColor: "rgba(0,0,255,0.1)",
      data: voltValues
    }]
  },
  options: {
    legend: {display: false},
    scales: {
      yAxes: [{ticks: {min: -1, max:1}}],
    }
  }
});

let currValues = [];
let timecurrValues = [];
let CurrChart = new Chart(document.getElementById("CURR-CHART"), {
  type: "line",
  data: {
    labels: timecurrValues,
    datasets: [{
      fill: false,
      lineTension: 0,
      backgroundColor: "rgba(0,0,255,1.0)",
      borderColor: "rgba(0,0,255,0.1)",
      data: currValues
    }]
  },
  options: {
    legend: {display: false},
    scales: {
      yAxes: [{ticks: {min: -250, max:250}}],
    }
  }
});

function addData(chart, label, newData) {
  if (chart.data.labels.length > 10)
    chart.data.labels.shift();
  chart.data.labels.push(label);
  if (chart.data.datasets[0].data.length > 10)
    chart.data.datasets[0].data.shift();
  chart.data.datasets.forEach((dataset) => {
    dataset.data.push(newData);
  });
  chart.update();
}

function setDisabled(id, value) {
  let element = document.getElementById(id);
  if (element) element.disabled = value;
}

function init() {
  Socket = new WebSocket("ws://" + window.location.hostname + ":81/");
  Socket.onmessage = function (event) {
    processCommand(event);
  };
}

function StateButton() {
  var message = {
    type: "CHANGE-STATE",
  };
  Socket.send(JSON.stringify(message));
}

function ReverseButton() {
  setDisabled("REVERSE-BUTTON", true);
  var message = {
    type: "CHANGE-ROTATION",
  };
  Socket.send(JSON.stringify(message));
}

function checkSpeed(value) {
  if (value > 100) return 100;
  if (value < 0) return 0;
  return value;
}

function UpdateRPMInput(value) {
  MotorSpeed = value;
  document.getElementById("RPM-INPUT").value = value;
}

function UpdateRPMInputButton() {
  var message = {
    type: "CHANGE-SPEED",
    speed: MotorSpeed,
  };
  Socket.send(JSON.stringify(message));
  document.getElementById("RPM-SLIDE").value = MotorSpeed;
  document.getElementById("RPM-INPUT").value = MotorSpeed;
  document.getElementById("RPM-SPAN").innerHTML = MotorSpeed;
}

function RPMSliderPointerDown(value) {
  StartRPMSliderValue = value;
}
function RPMSliderPointerUp(value) {
  MotorSpeed = value;
  UpdateRPMInputButton();
}
function RPMSliderPointerMove(value) {
  document.getElementById("RPM-SPAN").innerHTML = value;
  if (Math.abs(value - StartRPMSliderValue) % 10 == 0) {
    MotorSpeed = value;
    UpdateRPMInputButton();
  }
}

function UpdateHour(value) {
  TimeHour = value;
}
function UpdateMin(value) {
  TimeMin = value;
}
function UpdateSec(value) {
  TimeSec = value;
}
function UpdateTimerButton() {
  setDisabled("TIME-HOUR", true);
  setDisabled("TIME-MIN", true);
  setDisabled("TIME-SEC", true);
  setDisabled("TIME-BUTTON", true);
  setDisabled("TIME-DIS-BUTTON", false);
  let message = {
    type: "UPDATE-TIMER",
    hours: TimeHour,
    minutes: TimeMin,
    seconds: TimeSec,
  };
  Socket.send(JSON.stringify(message));
  console.log("SET TIMER " + TimeHour + ":" + TimeMin + ":" + TimeSec);
}
function DisableTimerButton() {
  setDisabled("TIME-HOUR", false);
  setDisabled("TIME-MIN", false);
  setDisabled("TIME-SEC", false);
  setDisabled("TIME-BUTTON", false);
  setDisabled("TIME-DIS-BUTTON", true);
  document.getElementById("TIME-HOUR").value = 0;
  document.getElementById("TIME-MIN").value = 0;
  document.getElementById("TIME-SEC").value = 0;
  TimeHour = 0;
  TimeMin = 0;
  TimeSec = 0;
  let message = {
    type: "DISABLE-TIMER",
  };
  Socket.send(JSON.stringify(message));
}

function processCommand(event) {
  let dt = new Date();
  document.getElementById("TIME").innerHTML = dt.toLocaleTimeString();
  document.getElementById("DATE").innerHTML = dt.toLocaleDateString();

  let jsonMessage = JSON.parse(event.data);
  
  let speed = jsonMessage.RPM;
  document.getElementById("RPM").innerHTML = speed;
  addData(RPMChart, dt.toLocaleTimeString(), speed);
  
  let volt = jsonMessage.VOLT;
  let curr = jsonMessage.CURR;
  document.getElementById("VOLT").innerHTML = volt;
  addData(VoltChart, dt.toLocaleTimeString(), volt);
  document.getElementById("CURR").innerHTML = curr;
  addData(CurrChart, dt.toLocaleTimeString(), curr);

  if (jsonMessage.MOTOR_STATE === "ON") {
    document.getElementById("STATE-BUTTON").innerHTML = "Стоп";
    setDisabled("REVERSE-BUTTON", false);
  }
  else {
    document.getElementById("STATE-BUTTON").innerHTML = "Старт";
    document.getElementById("RPM-SLIDE").value = 0;
    document.getElementById("RPM-INPUT").value = 0;
    document.getElementById("RPM-SPAN").value = 0;
    MotorSpeed = 0;
    setDisabled("REVERSE-BUTTON", true);
  }
  let timerState = jsonMessage.TIMER_STATE === "ON";
  setDisabled("TIME-HOUR", timerState);
  setDisabled("TIME-MIN", timerState);
  setDisabled("TIME-SEC", timerState);
  setDisabled("TIME-BUTTON", timerState);
  setDisabled("TIME-DIS-BUTTON", !timerState);
  document.getElementById("TIME-LEFT").innerHTML = jsonMessage.TIME_LEFT;
}

window.onload = function (event) {
  init();
};

