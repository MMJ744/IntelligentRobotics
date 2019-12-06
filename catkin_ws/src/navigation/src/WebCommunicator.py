#!/usr/bin/env python
from flask import Flask
from flask import request
import navTo
import navController
import navTo

import taskManager
import taskExecuter

app = Flask(__name__)

model = None


@app.route('/')
def hello():
    print("hello")
    return 'Hello World'

@app.route('/tasks')
def tasks():
    global model
    return model.output


@app.route('/kitchentask')
def kitchentask():
    print("kitchentask")
    try:
        user_int = int(request.args.get('table', ''))
        user_str = "table" + str(user_int)
        taskManager.new_task("Deliver", user_int)
        text = "waiter summoned for " + user_str
        model.prepend_message("kitchen", text)

        text = text + "\n\n" + model.messages["kitchen"]
        print("kitchentask")
    except:
        text = "error creating delivery task, please try again"
        print("kitchentask failed")

    return text


@app.route('/kitchen')
def kitchen():
    print("kitchen")

    text = model.messages["kitchen"]
    return text


@app.route('/staff')
def staff():
    print("staff")
    global model
    text = model.messages["staff"]
    return text


@app.route('/goto')
def goto():
    navTo.navigateTo(request.args.get('location',''))
    return 'going to ' + request.args.get('location', '')



def main(modelt):
    global model
    model = modelt
    app.run(host='0.0.0.0', debug=True)
