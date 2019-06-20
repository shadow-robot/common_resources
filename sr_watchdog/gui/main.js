// Connecting to ROS
// -----------------  

var system_name = $("#" + getElementIdByNameFromDiv("watchdog", "system_name_text"))
var system_status = $("#" + getElementIdByNameFromDiv("watchdog", "system_status_text"))
var progress_bar_text = $("#" + getElementIdByNameFromDiv("watchdog", "progress_bar_text"))
var progress_bar_percentage = $("#" + getElementIdByNameFromDiv("watchdog", "progress_bar_percentage"))
var failing_tests_textbox = $("#" + getElementIdByNameFromDiv("watchdog", "failing_tests_textbox"))
var logs_textbox = $("#" + getElementIdByNameFromDiv("watchdog", "logs_textbox"))
var led = $("#" + getElementIdByNameFromDiv("watchdog", "status_led"))

// failing_tests_textbox.text('chuj')

var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
});
    
ros.on('connection', function() {
    console.log('Connected to websocket server.');
});
    
ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
});
    
ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});
    
// Subscribing to a Topic
// ----------------------
    
var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/sr_watchdog',
    messageType : 'sr_watchdog/SystemStatus'
});
    
listener.subscribe(function(message) {
    system_name.val(message.system_name)

    if (-1 == message.status) {
        system_status.val("Pending")
    } else if (0 == message.status) {
        system_status.val("OK")
        led.css("background-color", "#ABFF00");
        led.css("box-shadow", "rgba(0, 0, 0, 0.2) 0 -1px 7px 1px, inset #304701 0 -1px 9px, #89FF00 0 2px 12px");
    } else {
        system_status.val("Error")
        led.css("background-color", "#F00");
        led.css("box-shadow", "rgba(0, 0, 0, 0.2) 0 -1px 7px 1px, inset #441313 0 -1px 9px, rgba(255, 0, 0, 0.5) 0 2px 12px");

    }

    progress_bar_text.text(message.checks_cycle_completion + '%')
    progress_bar_percentage.width(message.checks_cycle_completion + '%')

    var i, failing_tests_list;
    failing_tests_list = ""
    for (i = 0; i < message.test_statuses.length; i++) {
        if (!message.test_statuses[i].result) {
            failing_tests_list += message.test_statuses[i].test_name + "\n";
            if (0 == message.status) {
                led.css("background-color", "#FF0");
                led.css("box-shadow", "rgba(0, 0, 0, 0.2) 0 -1px 7px 1px, inset #808002 0 -1px 9px, #FF0 0 2px 12px");
            }
        }
    }
    if (!failing_tests_list) {
        failing_tests_textbox.css("visibility", "hidden");
    } else {
        failing_tests_textbox.css("visibility", "visible");
        failing_tests_textbox.text(failing_tests_list)
    }

    var log_list;
    log_list = ""
    for (i = 0; i < message.logs.length; i++) {
        if (0 == message.logs[i].type){
            log_list += message.logs[i].msg + "<br>";
        } else if (1 == message.logs[i].type){
            log_list += "<font color='red'>" + message.logs[i].msg + "</font><br>";
        } else {
            log_list += "<font color='orange'>" + message.logs[i].msg + "</font><br>";
        }
    }

    logs_textbox.html(log_list)


});

function getElementIdByNameFromDiv(div_id, element_name){
    var div_elements = document.getElementById(div_id).querySelectorAll('*');
    for (var i = 0; i<div_elements.length; i++) {
        if (div_elements[i].getAttribute("name") == element_name){
            return div_elements[i].id
        }
    }
}