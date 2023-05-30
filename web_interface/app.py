#!/usr/bin/env python3
import rospy
from flask import Flask, send_from_directory, Response
from flask_restful import Api, Resource, reqparse
from flask_cors import CORS #comment this on deployment
from api.HelloApiHandler import HelloApiHandler
from api.camera_handler import CameraHandler
from api.command_handler import CommandHandler

rospy.init_node("web_interface", anonymous="True")

#helpful tutorial https://towardsdatascience.com/build-deploy-a-react-flask-app-47a89a5d17d9
#helpful stackoverflow post: https://stackoverflow.com/questions/74090972/how-to-stream-video-images-from-python-app-to-a-remote-reactjs-webpage

app = Flask(__name__, static_url_path='', static_folder='frontend/build')
CORS(app) #comment this on deployment
api = Api(app)

cam = CameraHandler()
command_handler = CommandHandler()

@app.route("/camera")
def camera_display():
    return Response(cam.get_last_image(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/time")
def time_display():
    return Response(cam.get_time())

@app.route("/", defaults={'path':''})
def serve(path):
    return send_from_directory(app.static_folder,'index.html')

@app.route("/command_buttons", methods=["POST"])
def command_buttons():
    print("Command button hit")
    if request.form["initiate_bite"] == "initiate_bite":
        command_handler.send_command("initiate bite")
    elif request.form["acquire_bite"] == "acquire_bite":
        command_handler.send_command("acquire bite")
    elif request.form["emergency_stop"] == "emergency_stop":
        command_handler.send_command("emergency stop")
    elif request.form["end_meal"] == "end_meal":
        command_handler.send_command("meal over")

api.add_resource(HelloApiHandler, '/flask/hello')
