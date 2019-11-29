from flask import Flask
from flask import request
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
    return 'going to ' + request.args.get('location', '') + str(locs[request.args.get('location', '')])

if __name__ == '__main__':
    app.run(host='0.0.0.0:5000')