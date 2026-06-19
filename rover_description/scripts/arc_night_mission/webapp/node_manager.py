import os
import signal
import subprocess
import time
import threading

NODES = [
    'mission_manager',
    'dome_exit',
    'astronaut_searcher',
    'fuel_trail_follower',
    'oxygen_tank_navigator',
    'dome_return_navigator',
]

NODE_DISPLAY_NAMES = {
    'mission_manager': 'Mission Manager',
    'dome_exit': 'Dome Exit',
    'astronaut_searcher': 'Astronaut Searcher',
    'fuel_trail_follower': 'Fuel Trail Follower',
    'oxygen_tank_navigator': 'Oxygen Tank Navigator',
    'dome_return_navigator': 'Dome Return Navigator',
}

NODE_ORDER = ['mission_manager', 'dome_exit', 'astronaut_searcher',
              'fuel_trail_follower', 'oxygen_tank_navigator', 'dome_return_navigator']

LOG_LEVEL_MAP = {
    'INFO': 'INFO',
    'WARN': 'WARN',
    'WARNING': 'WARN',
    'ERROR': 'ERROR',
    'DEBUG': 'DEBUG',
}


class NodeManager:
    def __init__(self, workspace_dir, log_callback=None):
        self.workspace_dir = workspace_dir
        self.log_callback = log_callback or (lambda lvl, name, msg: None)
        self._processes = {}
        self._lock = threading.Lock()
        self._reader_threads = {}

    def _get_env(self):
        env = os.environ.copy()
        setup_path = os.path.join(self.workspace_dir, 'install', 'setup.bash')
        if os.path.exists(setup_path):
            proc = subprocess.run(
                ['bash', '-c', f'source {setup_path} && env'],
                capture_output=True, text=True, env=env
            )
            for line in proc.stdout.splitlines():
                if '=' in line:
                    key, val = line.split('=', 1)
                    env[key] = val
        return env

    def _read_output(self, node_name, stream):
        import re
        for line in iter(stream.readline, ''):
            line = line.rstrip('\n\r')
            if line:
                level = 'INFO'
                m = re.search(r'\[(INFO|WARN|WARNING|ERROR|DEBUG)\]', line)
                if m:
                    level = LOG_LEVEL_MAP.get(m.group(1), 'INFO')
                self.log_callback(level, node_name, line)
        stream.close()

    def start(self, node_name):
        if node_name not in NODES:
            return False
        with self._lock:
            if node_name in self._processes and self._processes[node_name]['process'].poll() is None:
                return False
            env = self._get_env()
            script_path = os.path.join(self.workspace_dir, f'{node_name}.py')
            if not os.path.exists(script_path):
                return False
            proc = subprocess.Popen(
                ['python3', script_path],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
                bufsize=1,
                universal_newlines=True,
            )
            self._processes[node_name] = {
                'process': proc,
                'started': time.time(),
                'name': node_name,
            }
            t = threading.Thread(target=self._read_output, args=(node_name, proc.stdout), daemon=True)
            self._reader_threads[node_name] = t
            t.start()
        return True

    def stop(self, node_name):
        with self._lock:
            if node_name not in self._processes:
                return False
            proc_info = self._processes[node_name]
            proc = proc_info['process']
            if proc.poll() is not None:
                del self._processes[node_name]
                return True
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    proc.wait()
            except ProcessLookupError:
                pass
            del self._processes[node_name]
        return True

    def get_status(self, node_name):
        with self._lock:
            if node_name not in self._processes:
                return {
                    'name': node_name,
                    'display_name': NODE_DISPLAY_NAMES.get(node_name, node_name),
                    'running': False,
                    'pid': None,
                    'uptime': None,
                }
            proc_info = self._processes[node_name]
            proc = proc_info['process']
            running = proc.poll() is None
            return {
                'name': node_name,
                'display_name': NODE_DISPLAY_NAMES.get(node_name, node_name),
                'running': running,
                'pid': proc.pid if running else None,
                'uptime': time.time() - proc_info['started'] if running else None,
            }

    def get_all_statuses(self):
        ordered = []
        for name in NODE_ORDER:
            ordered.append(self.get_status(name))
        return ordered

    def stop_all(self):
        for name in list(NODES):
            self.stop(name)
