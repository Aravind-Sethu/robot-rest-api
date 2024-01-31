#!/usr/bin/env python
import argparse
import rospy
from actionlib_msgs.msg import GoalStatusArray

from flask import Flask
from flask_restful import Resource, Api

robot_status_data = {}
robot_status_data["data_received"] = False

def move_base_status_callback(data):
	"""Callback function for ROS topic /move_base/status"""
	if data.status_list:
		robot_status_data["status"] = data.status_list[0].status
		robot_status_data["text"] = data.status_list[0].text
		robot_status_data["data_received"] = True

class RobotStatus(Resource):
	"""Handler for /api/robot/status endpoint"""
	@staticmethod
	def get():
		"""GET request handler"""
		if robot_status_data["data_received"]:
			robot_status_data["data_received"] = False
			response = {
				"status": robot_status_data["status"],
				"text": robot_status_data["text"]
			}, 200
		else:
			response = {"message": "Invalid status value"}, 400
		return response

rospy.init_node("ros_rest_server", disable_signals=True)
rospy.Subscriber(
"/move_base/status", GoalStatusArray, move_base_status_callback
)

app = Flask("Robot Status REST Server")
api = Api(app)
api.add_resource(RobotStatus, "/api/robot/status")

if __name__ == "__main__":
	server_parser = argparse.ArgumentParser(description="Robot Status REST Server")
	server_parser.add_argument(
	"port", nargs="?", default="7201", type=int, help="Port number for the server"
	)
	server_parser.add_argument(
	"ip", nargs="?", default="127.0.0.1", help="IP address for the server"
	)
	args = server_parser.parse_args(rospy.myargv()[1:])
	port_number = args.port
	ip_address = args.ip
	app.run(ip_address, port=port_number)
