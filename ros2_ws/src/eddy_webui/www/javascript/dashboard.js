
function onLoadROS(ros) {
	$("#ros-halt").click(function() {
		shutdown_service = new ROSLIB.Service({
			ros : ros,
			name : '/system/shutdown',
			serviceType : 'std_srvs/Empty'
		});
		shutdown_service.callService(new ROSLIB.ServiceRequest({}), function () {});
	});
}

       function updateRosStatus(status) {
                if (status == 'connection') {
                        $("#ros-status").html("open");
                        $("#ros-status").addClass("bg-success");
                        $("#ros-status").removeClass("bg-danger");
                        $("#ros-status").removeClass("bg-secondary");
                } else if (status == 'error') {
                        $("#ros-status").html("error");
                        $("#ros-status").addClass("bg-danger");
                        $("#ros-status").removeClass("bg-success");
                        $("#ros-status").removeClass("bg-secondary");
                } else if (status == 'close') {
                        $("#ros-status").html("closed");
                        $("#ros-status").addClass("bg-secondary");
                        $("#ros-status").removeClass("bg-success");
                        $("#ros-status").removeClass("bg-danger");
                }
        }

function buildDashboard(name) {
	const names = ["Dashboard", "Camera", "GPS", "IMU", "Sonar", "Light", "Thrusters", "Battery", "", "Operational Mode", "Mission Planner", "Telemetry", "Diagnostics", "Vegetation", "Training"];
	const files = ["index.html", "camera.html", "gps.html", "imu.html", "sonar.html", "light.html", "thrusters.html", "battery.html", "", "operational_mode.html",  "mission_planner.html", "telemetry.html", "diagnostics.html", "vegetation.html", "training.html"];
	const icons = ["home", "camera", "map-pin", "compass", "bar-chart-2", "sun", "aperture", "battery", "", "sliders", "clipboard", "activity", "thermometer", "figma", "dribbble"];

	var list = document.getElementById("main_nav");
	for (var i = 0; i < names.length; i++) {
		if (names[i] == "") {
			list = document.getElementById("extra_nav");
			continue;
		}
		var link = document.createElement("li");
		link.classList.add("nav-item");

		var item = document.createElement("a");
		item.setAttribute("href", "./" + files[i])
		item.classList.add("nav-link");
		if (names[i] == name) {
			item.classList.add("active");
		}

		var span = document.createElement("span");
		span.setAttribute("data-feather", icons[i]);

		item.appendChild(span);

		item.innerHTML += names[i];

		link.appendChild(item);

		list.appendChild(link);
	}
	return;
}
