from requests import request
from flask import Flask
import random
from flask_cors import CORS
import time
import math

app = Flask(__name__)
CORS(app)

x = 257.0
prev_x = 257.0
y = 915.0
prev_y  = 915.0
angle = 0.0
speed = 0.0
order = "move"
status = "moving"
alert = False

t = time.time()
prev_t = time.time()

@app.route("/data")
def get_data():
    return {"x": x, "y": y, "angle": angle, "speed": speed, "order": order, "status": status, "alert": alert}

@app.route("/pos", methods=['POST', 'GET'])
def position():
    if request.method == 'POST':
        x = request.args.get('x', '')
        y = request.args.get('y', '')
        angle = request.args.get('angle')
        t = time.time()
        speed = math.sqrt((prev_x - x)**2 + (prev_y - y)**2) / (t - prev_t)
        prev_t = t
        prev_y = y
        prev_x = x
        alert = request.args.get('alert', '')