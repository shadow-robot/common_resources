var led_color = {
    GREEN: 0,
    RED: 1,
    YELLOW: 2,
  };

var system_status_code = {
    PENDING: -1,
    OK: 0,
    ERROR: 1,
  };

var log_type_code = {
    INFO: 0,
    ERROR: 1,
    WARN: 2,
  };

var system_name = $("#" + getElementIdByNameFromDiv("watchdog", "system_name_text"))
var system_status = $("#" + getElementIdByNameFromDiv("watchdog", "system_status_text"))
var progress_bar_text = $("#" + getElementIdByNameFromDiv("watchdog", "progress_bar_text"))
var progress_bar_percentage = $("#" + getElementIdByNameFromDiv("watchdog", "progress_bar_percentage"))
var failing_checks_textbox = $("#" + getElementIdByNameFromDiv("watchdog", "failing_checks_textbox"))
var logs_textbox = $("#" + getElementIdByNameFromDiv("watchdog", "logs_textbox"))
var led = $("#" + getElementIdByNameFromDiv("watchdog", "status_led"))

// ROSLIB HANDLING

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
 
var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/sr_watchdog',
    messageType : 'sr_watchdog/SystemStatus'
});
    
listener.subscribe(function(message) {
    system_name.val(message.system_name)
    change_system_status(message.status, system_status, led)

    progress_bar_text.text(message.checks_cycle_completion + '%')
    progress_bar_percentage.width(message.checks_cycle_completion + '%')

    handle_failing_checks_list(message, failing_checks_textbox, led)
    handle_watchdog_logs(message, logs_textbox)
});

// HELPER METHODS

function getElementIdByNameFromDiv(div_id, element_name){
    var div_elements = document.getElementById(div_id).querySelectorAll('*');
    for (var i = 0; i<div_elements.length; i++) {
        if (div_elements[i].getAttribute("name") == element_name){
            return div_elements[i].id
        }
    }
}

function change_led_color(led_id, color){
    if (led_color.GREEN == color){
        led_id.css("background-color", "#ABFF00");
        led_id.css("box-shadow", "rgba(0, 0, 0, 0.2) 0 -1px 7px 1px, inset #304701 0 -1px 9px, #89FF00 0 2px 12px");
    } else if (led_color.RED == color){
        led_id.css("background-color", "#F00");
        led_id.css("box-shadow", "rgba(0, 0, 0, 0.2) 0 -1px 7px 1px, inset #441313 0 -1px 9px, rgba(255, 0, 0, 0.5) 0 2px 12px");
    } else if (led_color.YELLOW == color){
        led.css("background-color", "#FF0");
        led.css("box-shadow", "rgba(0, 0, 0, 0.2) 0 -1px 7px 1px, inset #808002 0 -1px 9px, #FF0 0 2px 12px");
    } else{
        throw "Unsupported led color"
    }
}

function change_system_status(status_value, system_status_label, system_status_led){
    if (system_status_code.PENDING == status_value) {
        system_status_label.val("Pending")
    } else if (system_status_code.OK == status_value) {
        system_status_label.val("OK")
        change_led_color(system_status_led, led_color.GREEN)
    } else if (system_status_code.ERROR == status_value){
        system_status_label.val("Error")
        change_led_color(system_status_led, led_color.RED)
    } else {
        throw "Unsupported system status code"
    }
}

function handle_failing_checks_list(watchdog_message, failing_checks_textbox, status_led){
    var failing_checks_list = "";
    for (var i = 0; i < watchdog_message.check_statuses.length; i++) {
        if (!watchdog_message.check_statuses[i].result) {
            failing_checks_list += "&rarr; " + watchdog_message.check_statuses[i].check_name
                                    + " (" + watchdog_message.check_statuses[i].component + ")\n";
            if (0 == watchdog_message.status) {
                change_led_color(status_led, led_color.YELLOW)
            }
        }
    }
    if (!failing_checks_list) {
        failing_checks_textbox.css("visibility", "hidden");
    } else {
        failing_checks_textbox.css("visibility", "visible");
        failing_checks_textbox.html(failing_checks_list)
    }
}

function handle_watchdog_logs(watchdog_message, log_list_textbox){
    var log_list = ""
    for (var i = 0; i < watchdog_message.logs.length; i++) {
        if (log_type_code.INFO == watchdog_message.logs[i].type){
            log_list += watchdog_message.logs[i].msg + "<br>";
        } else if (log_type_code.ERROR == watchdog_message.logs[i].type){
            log_list += "<font color='red'>" + watchdog_message.logs[i].msg + "</font><br>";
        } else if(log_type_code.WARN) {
            log_list += "<font color='orange'>" + watchdog_message.logs[i].msg + "</font><br>";
        } else {
            throw "Unsupported log type code"
        }
    }
    log_list_textbox.html(log_list)
}