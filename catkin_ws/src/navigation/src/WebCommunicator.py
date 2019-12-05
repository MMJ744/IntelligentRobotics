#!/usr/bin/env python
from flask import Flask
from flask import request
import navController
import navTo

import taskManager
import taskExecuter

app = Flask(__name__)


@app.route('/')
def hello():
    print("hello")
    return 'Hello'


@app.route('/kitchen')
def kitchen():
    try:
        user_int = int(request.args.get('table', ''))
        user_str = "table" + str(user_int)
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
    navTo.navigateTo(request.args.get('location',''))
    return 'going to ' + request.args.get('location', '')


if __name__ == '__main__':
    # navTo.main()

    app.run(host='0.0.0.0')
