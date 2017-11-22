#!/usr/bin/env python
from flask import Flask, render_template, session, request
from util import success_msg, error_msg
from flask_cors import CORS
from flask_socketio import SocketIO, emit, disconnect
import webbrowser
import xmlrpclib
import sys

# Set this variable to "threading", "eventlet" or "gevent" to test the
# different async modes, or leave it set to None for the application to choose
# the best option based on installed packages.
async_mode = None

app = Flask(__name__)
CORS(app)
app.config['SECRET_KEY'] = 'secret!'
app.config.update(TEMPLATES_AUTO_RELOAD=True)
socketio = SocketIO(app, async_mode=async_mode)

cahce_size = 10
cache = []
pointer = -1


def _push_data(data):
    global cache, pointer
    while len(cache) > cahce_size:
        cache.pop(0)
        pointer -= 1
    cache.append(data)
    pointer += 1


@app.route('/imshow', methods=['POST'])
def imshow():
    req_json = request.get_json(force=True)
    # if 'b64' in req_json:
    #     b64 = req_json['b64']
    # else:
    #     return error_msg("miss argument b64")
    # if 'json' in req_json:
    #     j = req_json['json']
    # else:
    #     return error_msg("miss argument json")
    try:
        # data = {'data': b64, 'json': j}
        socketio.emit('my_response', req_json, namespace='/test')
        _push_data(req_json)
    except KeyError as e:
        print e
        return error_msg(str(e))
    return success_msg("success")


@app.route('/')
def index():
    return render_template('index.html', async_mode=socketio.async_mode)


@app.route('/map')
def map_index():
    return render_template('map.html', async_mode=socketio.async_mode)


@app.route('/mapshow', methods=['POST'])
def mapshow():
    req_json = request.get_json(force=True)
    print req_json
    try:
        socketio.emit('map_show', req_json, namespace='/map')
    except KeyError as e:
        print e
        return error_msg(str(e))
    return success_msg("success")


@socketio.on('play', namespace='/test')
def pause_play():
    print "play"
    proxy = xmlrpclib.ServerProxy("http://localhost:25000")
    return proxy.pause_play()


@socketio.on('next', namespace='/test')
def next_image():
    global pointer
    print "next"
    if pointer < len(cache) - 1:
        pointer += 1
        try:
            socketio.emit('my_response', cache[pointer], namespace='/test')
        except KeyError as e:
            print e
            return False
        return True
    proxy = xmlrpclib.ServerProxy("http://localhost:25000")
    return proxy.next_image()


@socketio.on('prev', namespace='/test')
def prev_image():
    global pointer
    print "prev"
    if pointer > 0:
        pointer -= 1
        try:
            socketio.emit('my_response', cache[pointer], namespace='/test')
        except KeyError as e:
            print e
            return False
        return True
    return False


@socketio.on('tag', namespace='/test')
def tag(message):
    print "tag"
    proxy = xmlrpclib.ServerProxy("http://localhost:25000")
    return proxy.tag(message)


@app.route('/hello')
def hello():
    return "hello"


@socketio.on('disconnect_request', namespace='/test')
def disconnect_request():
    session['receive_count'] = session.get('receive_count', 0) + 1
    emit('my_response',
         {'data': 'Disconnected!', 'count': session['receive_count']})
    disconnect()


@socketio.on('connect', namespace='/test')
def test_connect():
    print "connect"

if __name__ == '__main__':
    print sys.argv
    if len(sys.argv) > 1:
        if sys.argv[1] == 'map':
            webbrowser.open('http://localhost:25001/map', autoraise=True)
    else:
        webbrowser.open('http://localhost:25001', autoraise=True)
    socketio.run(app, host='0.0.0.0', port=25001)
