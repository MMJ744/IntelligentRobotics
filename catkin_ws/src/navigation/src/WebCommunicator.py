#!/usr/bin/env python
from flask import Flask
from flask import request
import navTo

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
        table = request.args.get('table', '')
        taskManager.new_task("Deliver", table)
        text = "waiter summoned for table " + table
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
    navTo.navigate(request.args.get('location',''))
    return 'going to ' + request.args.get('location', '') + str(locs[request.args.get('location', '')])


if __name__ == '__main__':
    # navTo.main()

    app.run(host='0.0.0.0')
