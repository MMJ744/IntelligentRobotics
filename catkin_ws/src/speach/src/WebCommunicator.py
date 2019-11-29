#!/usr/bin/env python
from flask import Flask
from flask import request
import navcall

app = Flask(__name__)

locs = {
    'table1': {
        'x' : 10,
        'y': 20
    },
    'table2': {
        'x': 59,
        'y':23
    }
}

@app.route('/')
def hello():
    return 'Hello'

@app.route('/goto')
def goto():
    navcall.call(request.args.get('location',''))
    return 'going to ' + request.args.get('location', '') + str(locs[request.args.get('location', '')])

if __name__ == '__main__':
    navcall.main()
    app.run(host='0.0.0.0')