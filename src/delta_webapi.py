from subprocess import Popen
import time
import psutil
import tempfile
import json

import delta_utils

from http.server import BaseHTTPRequestHandler, HTTPServer
import json

class DeltaWebApiHandler(BaseHTTPRequestHandler):
       
    player_proc = None
    
    def do_GET(self):
        try:
            teaching_list = ['/delta_teaching_offline', '/delta_teaching_online', '/delta_teaching_direct']
            if self.path in teaching_list:
                proc_name = self.path + '.py'
                if self.is_playing():
                    response = {
                        'result': 'NG',
                        'reason': 'Now Playing'
                    }
                    status_code = 409
                else:
                    self.start_teaching(proc_name)
                    response = {
                        'result': 'OK'
                    }
                    status_code = 200

            elif self.path == '/status':
                status = 'Idle'
                if self.is_playing():
                    status = 'Playing'
                response = {
                    'result': 'OK',
                    'status': status
                }
                status_code = 200
            else:
                response = {
                    'result': 'NG',
                    'reason': 'Invalid URL'
                }
                status_code = 404

        except Exception as e:
            response = {
                'result': 'Error',
                'reason': str(e)
            }
            status_code = 500
        
        self.send_response(status_code)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(response).encode('UTF-8'))
 

    def do_POST(self):
        try:
            if self.path == '/delta_player_offline':
                proc_name = self.path + '.py'
            elif self.path == '/delta_player_online':
                proc_name = self.path + '.py'
            else:
                raise Exception('Invalid URL')

            response = None
            status_code = 500
    
            if self.is_playing():
                response = {
                    'result': 'NG',
                    'reason': 'Now Playing'
                }
                status_code = 409
            else:
                content_len = int(self.headers.get('content-length'))
                history = json.loads(self.rfile.read(content_len).decode('utf-8'))
                history_filename = self.convert_to_playablefile(history)
                self.start_playing(proc_name, history_filename)
                response = {
                    'result': 'OK'
                }
                status_code = 200
        except Exception as e:
            response = {
                'result': 'Error',
                'reason': str(e)
            }
            status_code = 500

        self.send_response(status_code)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(response).encode('UTF-8'))

    def start_playing(self, proc_name, history_file):
        DeltaWebApiHandler.player_proc = Popen(['python', './' + proc_name, history_file])
        player_pid = DeltaWebApiHandler.player_proc.pid
        proc = psutil.Process(pid=player_pid)
        proc.nice(psutil.REALTIME_PRIORITY_CLASS)

    def start_teaching(self, proc_name):
        DeltaWebApiHandler.player_proc = Popen(['python', './' + proc_name])
        player_pid = DeltaWebApiHandler.player_proc.pid
        proc = psutil.Process(pid=player_pid)
        proc.nice(psutil.REALTIME_PRIORITY_CLASS)

    def is_playing(self):
        if (DeltaWebApiHandler.player_proc is not None) and (DeltaWebApiHandler.player_proc.poll() is None):
            return True
        else:
            return False
    
    def convert_to_playablefile(self, history):
        playable_history = {
            "type": "hand_position",
            "data": []
        }
        playable_history['data'] = delta_utils.linear_interpolation(history['data'])
        for history_data in playable_history['data']:
            if len(history_data) == 4:
                history_data.append(60)
        self.player_file = open('./tmp/history_file', 'w')
        json.dump(playable_history, self.player_file)
        self.player_file.close() 
        print(self.player_file.name)
        return self.player_file.name

try:
    http = HTTPServer(('localhost', 8000), DeltaWebApiHandler)
    print('start httpserver')
    http.serve_forever()
except Exception as e:
    print(e)

