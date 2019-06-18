// Connecting to ROS
// -----------------  

var system_name = $("#" + getElementIdByNameFromDiv("watchdog", "system_name_text"))
var system_status = $("#" + getElementIdByNameFromDiv("watchdog", "system_status_text"))
var progress_bar_text = $("#" + getElementIdByNameFromDiv("watchdog", "progress_bar_text"))
var progress_bar_percentage = $("#" + getElementIdByNameFromDiv("watchdog", "progress_bar_percentage"))
var failing_tests_textbox = $("#" + getElementIdByNameFromDiv("watchdog", "failing_tests_textbox"))

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
    console.log('Received message on ' + listener.name);
    system_name.val(message.system_name)
    system_status.val(message.status)
    progress_bar_text.text(message.checks_cycle_completion + '%')
    progress_bar_percentage.width(message.checks_cycle_completion + '%')

    var i, failing_tests_list;
    failing_tests_list = ""
    for (i = 0; i < message.test_statuses.length; i++) {
        if (!message.test_statuses[i].result)
        {
            failing_tests_list += message.test_statuses[i].test_name + "\n";
        }
    }
    failing_tests_textbox.text(failing_tests_list)

});

function getElementIdByNameFromDiv(div_id, element_name){
    var div_elements = document.getElementById(div_id).querySelectorAll('*');
    for (var i = 0; i<div_elements.length; i++) {
        if (div_elements[i].getAttribute("name") == element_name){
            return div_elements[i].id
        }
    }
}