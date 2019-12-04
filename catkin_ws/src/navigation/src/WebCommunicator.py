#!/usr/bin/env python
from flask import Flask
from flask import request
import navController

import taskManager
import taskExecuter

app = Flask(__name__)

locs = {
    'table1': {
        'x': 10,
        'y': 20
    },
    'table2': {
        'x': 59,
        'y': 23
    }
}


@app.route('/')
def hello():
    print("hello")
    return 'Hello'


@app.route('/kitchen')
def kitchen():
    try:
        user_int = int(request.args.get('table', ''))
        user_str = "table" + str(user_int)
        if user_str in navController.locations.keys:
            taskManager.new_task("Deliver", user_int)
        text = "waiter summoned for " + user_str
        taskExecuter.send_message("kitchen", text)
    except:
        text = "call with /kitchen?table=<table_number>"

    text = text + "\n\n" + taskExecuter.messages("kitchen")
    return text


@app.route('/staff')
def staff():
    text = taskExecuter.messages("staff")
    return text


@app.route('/goto')
def goto():
    navController.navigateTo(request.args.get('location',''))
    return 'going to ' + request.args.get('location', '') + str(locs[request.args.get('location', '')])


if __name__ == '__main__':
    # navTo.main()

    app.run(host='0.0.0.0')
