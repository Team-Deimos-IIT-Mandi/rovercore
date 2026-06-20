import os
from flask import Flask, jsonify, render_template, Response, request
from ros_bridge import ROSBridge, MISSION_STATES, STATE_TRANSITIONS, DEFAULT_CAMERA_TOPICS
from node_manager import NodeManager, NODES, NODE_DISPLAY_NAMES, NODE_ORDER

app = Flask(__name__)
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0
app.config['MISSION_NAME'] = os.environ.get('MISSION_NAME', 'ARC NIGHT')


@app.context_processor
def inject_globals():
    return {'mission_name': app.config['MISSION_NAME']}

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ros_bridge = ROSBridge()
node_manager = NodeManager(BASE_DIR, log_callback=ros_bridge.add_log)

bridge_started = False


def ensure_bridge():
    global bridge_started
    if not bridge_started:
        ros_bridge.start()
        bridge_started = True


@app.route('/')
def index():
    ensure_bridge()
    return render_template('index.html',
                           states=MISSION_STATES,
                           transitions=STATE_TRANSITIONS,
                           nodes=NODES,
                           node_display_names=NODE_DISPLAY_NAMES,
                           node_order=NODE_ORDER)


@app.route('/about')
def about():
    return render_template('about.html',
                           transitions=STATE_TRANSITIONS,
                           node_display_names=NODE_DISPLAY_NAMES,
                           node_order=NODE_ORDER,
                           default_camera_topics=DEFAULT_CAMERA_TOPICS)


@app.route('/api/state')
def api_state():
    ensure_bridge()
    current = ros_bridge.get_state()
    transitions = STATE_TRANSITIONS.get(current, None)
    return jsonify({
        'state': current,
        'can_advance': transitions is not None,
        'next_state': transitions[1] if transitions else None,
        'valid_states': MISSION_STATES,
    })


@app.route('/api/nodes')
def api_nodes():
    ensure_bridge()
    statuses = node_manager.get_all_statuses()
    return jsonify(statuses)


@app.route('/api/node/<name>/start', methods=['POST'])
def api_node_start(name):
    ensure_bridge()
    success = node_manager.start(name)
    return jsonify({'success': success})


@app.route('/api/node/<name>/stop', methods=['POST'])
def api_node_stop(name):
    ensure_bridge()
    success = node_manager.stop(name)
    return jsonify({'success': success})


@app.route('/api/node/stop_all', methods=['POST'])
def api_node_stop_all():
    ensure_bridge()
    node_manager.stop_all()
    return jsonify({'success': True})


@app.route('/api/mission/advance', methods=['POST'])
def api_mission_advance():
    ensure_bridge()
    current = ros_bridge.get_state()
    if current in STATE_TRANSITIONS:
        ros_bridge.advance_state()
        return jsonify({'success': True, 'from': current, 'next': STATE_TRANSITIONS[current][1]})
    return jsonify({'success': False, 'reason': f'No transition available from {current}'})


@app.route('/api/mission/estop', methods=['POST'])
def api_mission_estop():
    ensure_bridge()
    ros_bridge.emergency_stop()
    return jsonify({'success': True})


@app.route('/api/logs')
def api_logs():
    ensure_bridge()
    logs = ros_bridge.get_logs(n=200)
    return jsonify(logs)


@app.route('/api/camera/<label>')
def api_camera(label):
    ensure_bridge()
    frame = ros_bridge.get_camera_frame(label)
    if frame:
        return Response(frame, mimetype='image/jpeg')
    return Response(status=204)


@app.route('/api/cameras')
def api_cameras():
    ensure_bridge()
    return jsonify(ros_bridge.get_camera_topics())


@app.route('/api/cameras/add', methods=['POST'])
def api_cameras_add():
    ensure_bridge()
    data = request.get_json()
    label = data.get('label', '').strip()
    topic = data.get('topic', '').strip()
    if not label or not topic:
        return jsonify({'success': False, 'reason': 'Label and topic are required'}), 400
    ok = ros_bridge.add_camera_topic(label, topic)
    if not ok:
        return jsonify({'success': False, 'reason': f'Label "{label}" already exists'}), 400
    return jsonify({'success': True, 'label': label, 'topic': topic})


@app.route('/api/cameras/remove', methods=['POST'])
def api_cameras_remove():
    ensure_bridge()
    data = request.get_json()
    label = data.get('label', '').strip()
    if not label:
        return jsonify({'success': False, 'reason': 'Label is required'}), 400
    ros_bridge.remove_camera_topic(label)
    return jsonify({'success': True, 'label': label})


@app.route('/api/cameras/update', methods=['POST'])
def api_cameras_update():
    ensure_bridge()
    data = request.get_json()
    label = data.get('label', '').strip()
    topic = data.get('topic', '').strip()
    if not label or not topic:
        return jsonify({'success': False, 'reason': 'Label and topic are required'}), 400
    ros_bridge.update_camera_topic(label, topic)
    return jsonify({'success': True, 'label': label, 'topic': topic})


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)