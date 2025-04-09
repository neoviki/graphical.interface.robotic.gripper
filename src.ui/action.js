/* async system call */
const spawnSync = require('child_process').spawnSync;

var SerialPort = require('serialport');
var ser_port;
var read_ine;
var serial_parser;
var debug = true;
var port_list = [];
var IN_PROGRESS = 1;
var CONNECTED = 2;
var DISCONNECTED = 3;
var serial_status = DISCONNECTED;

function add_port(port, index)
{
	port_list[index] = port;
	/* Since add_port will be called asynchronously,
	 * we are generating dropdown list here 
	 */
	html_dropdown_list_clean_insert(port_list);
	html_button_enable("id_btn_scan");
}

/*Port list will be added to port_list[] asynchronously*/
function get_serial_ports(){
	let i = 0;
	/* The list will be returned asynchronously */
	SerialPort.list(function (err, ports) {
		ports.forEach(function(port) {
			if(debug == true){
				message = "[port : "+i+"] [ "+port.comName+" ]"
				console.log(message);
			}
			add_port(port.comName, i)
			i += 1;
		});
	});
}

function open_serial_port(port_name){
	if(debug == true){
		msg = "Opening Serial Monitor : " + port_name;
		console.log(msg);
	}
	ser_port = new SerialPort(port_name, 9600);
	readline = SerialPort.parsers.Readline;
	serial_parser = new readline();
	ser_port.pipe(serial_parser);

	ser_port.on("open", serial_open);
	serial_parser.on("data", html_serial_data_display);
	ser_port.on("close", serial_close);
	ser_port.on("error", serial_error);

}

function serial_open()
{
	if(debug == true){
		message = "serial port opened";
		console.log(message);
	}

	document.getElementById("id_btn_connect").innerHTML = "Disconnect";
	serial_status = CONNECTED;
	html_button_enable("id_btn_connect");
	html_textarea_clear('id_serial_reader');
	html_textarea_append('id_serial_reader', "[ serial port connected ]\n");
	html_textarea_append('id_serial_reader', " ----------------------- \n");
}

function serial_close()
{
	message = "serial port closed"; 
	if(debug == true){
		console.log(message);
	}
	//alert(message);
	document.getElementById("id_btn_connect").innerHTML = "Connect";
	serial_status = DISCONNECTED;
	html_button_enable("id_btn_connect");
	scan_serial_ports();
	html_textarea_append('id_serial_reader', "[ serial port disconnected ]\n");
}

function serial_error(error)
{
	alert(error);
	document.getElementById("id_btn_connect").innerHTML = "Connect";
	serial_status = DISCONNECTED;
	html_button_enable("id_btn_connect");
	scan_serial_ports();
}

function html_input_clear(id)
{
    document.getElementById(id).value = "";
}

function html_input_read(id)
{
    data = document.getElementById(id).value;
	return data;
}

function html_textarea_clear(id)
{
    document.getElementById(id).value = "";
	html_textarea_scroll(id);
}

function html_textarea_read(id){
    data = document.getElementById(id).value;
	return data;
}

/*overwrite existing contents*/
function html_textarea_overwrite(id, data)
{
	html_textarea_clear(id);
    document.getElementById(id).value = data;
	html_textarea_scroll(id);
}

function html_textarea_append(id, data)
{
    document.getElementById(id).value += data;
	html_textarea_scroll(id);
}

/* autoscroll text area on feed */
function html_textarea_scroll(id){
    document.getElementById(id).scrollTop = document.getElementById(id).scrollHeight
}



function html_serial_data_display(data)
{
	/*write to text area*/
    html_textarea_append('id_serial_reader', data);
}

function html_insert_between_tags(id, data)
{
	document.getElementById(id).innerHTML = data;
}

function html_clear_dropdown_list()
{
	if(debug == true){
		console.log("html_clear_dropdown_list()");
	}

	/*Crear Port List*/
	port_list = [];
	list = '';
	html_insert_between_tags("id_select_serial_ports", list);
}
function html_button_disable(id)
{
    document.getElementById(id).disabled = true;
}

function html_button_enable(id){
    document.getElementById(id).disabled = false;
}

function html_dropdown_list_clean_insert(array)
{
	if(debug == true){
		console.log("html_dropdown_list_clean_insert()");
	}

	list = '';
	list += '<option selected>\< select serial port \></option>';

	for(i=0; i<array.length; i++){
		list += '<option>'+array[i]+'</option>';
		if(debug == true){
			msg = "[ port : "+array[i]+" ]";
			console.log(msg);
		}
	}

	drop_down_id = "id_select_serial_ports"
	html_insert_between_tags(drop_down_id, list);
}

function scan_serial_ports(){
	if(debug == true){
		console.log("scan_serial_ports()");
	}

	html_button_disable("id_btn_scan");

	/*Crear Port List*/
	port_list = [];

	html_clear_dropdown_list();

	list = '';
	list += '<option selected>\< scanning ...\></option>';

	html_insert_between_tags("id_select_serial_ports", list);
	get_serial_ports();
}

function send_serial_data(data)
{
	ser_port.write(data, function(error) {
		if (error) {
			alert("serial write error");
			console.log('error serial write()', error.message);
			return;
		}
		console.log('sent data via serial port');
	});
}

function check_serial_port_status(){
	if(serial_status == IN_PROGRESS){
		alert("serial port is being connected, try sending after some time");
		return -1;
	}

	if(serial_status != CONNECTED){
		alert("no serial port connected");
		return -1;
	}

	return 0;
}

function event_input_gripper_range()
{
	if(debug == true){
		console.log("event_input_gripper_range()");
	}
	value = html_input_read("id_gripper_range");
	data = value+"%";
	html_insert_between_tags("id_gripper_percentage_value", data);
	serial_command = "P"+value+"\n";

	if( check_serial_port_status() < 0 ){
		return;
	}

	send_serial_data(serial_command);
}

function event_btn_gripper_open()
{
	if(debug == true){
		console.log("event_btn_gripper_open()");
	}

	if(check_serial_port_status() < 0){
		return;
	}

	serial_command = "o\n";
	send_serial_data(serial_command);
}

function event_btn_gripper_close()
{
	if(debug == true){
		console.log("event_btn_gripper_close()");
	}

	if(check_serial_port_status()<0){
		return;
	}

	serial_command = "c\n";
	send_serial_data(serial_command);
}

function event_btn_send_sp()
{
	if(debug == true){
		console.log("event_btn_send_sp()");
	}

	if(check_serial_port_status()<0){
		return;
	}

	html_button_disable("id_btn_send");
	data = html_input_read("id_serial_writer");
	data = data + "\n";

	if(debug == true){
		message = "serial_write : "+data;
		console.log(message);
	}

	send_serial_data(data);
	html_input_clear("id_serial_writer");
	html_button_enable("id_btn_send");
}


function event_btn_scan_sp()
{
	if(debug == true){
		console.log("event_btn_scan_sp()");
	}
	html_button_disable("id_btn_scan");
	scan_serial_ports();
}

function connect_serial_port()
{
	port = document.getElementById("id_select_serial_ports");
	selected_port = port.options[port.selectedIndex].value;

	if(debug == true){
		message = "[ SELECTED PORT : " + selected_port + " ] [ Index : " + port.selectedIndex + " ]";
		console.log(message);
	}

	if(selected_port == "< select serial port >"){
		alert("select proper serial port from the dropdown");
		serial_status = DISCONNECTED;
		html_button_enable("id_btn_connect");
		return;
	}

	open_serial_port(selected_port);
}

function event_btn_connect_sp()
{
	if(debug == true){
		console.log("event_btn_connect_sp()");
	}

	html_button_disable("id_btn_connect");

	if(serial_status == IN_PROGRESS){
		return;
	}else if(serial_status == CONNECTED){
		serial_status = IN_PROGRESS;
		disconnect_serial_port();
		return;
	}else if(serial_status == DISCONNECTED){
		serial_status = IN_PROGRESS;
		connect_serial_port();
		return;
	}

}

function disconnect_serial_port()
{
	if(debug == true){
		console.log("disconnect_serial_port()");
	}

	ser_port.close();
	document.getElementById("id_btn_connect").innerHTML = "Connect";
	serial_status = DISCONNECTED;
    html_button_enable("id_btn_connect");

}

function sleep(milliseconds){
	return new Promise(resolve=> setTimeout(resolve, milliseconds));
}

async function sleep_test(){
	alert("Going to sleep for 10 seconds");
	await sleep(10000);
	alert("Woke up after 10 seconds of nice rest!!");
}

scan_serial_ports();
