from flask import Flask, request, jsonify
import requests
import xml.etree.ElementTree as ET
from pyproj import Proj, transform, Transformer
from flask_cors import CORS
import json
import cx_Oracle
import logging  # 로깅을 위한 모듈 임포트

from urllib.parse import quote


app = Flask(__name__)
CORS(app, supports_credentials=True, origins=['http://localhost:3000'])


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger()

# 캐시를 저장할 딕셔너리 초기화
cache = {}


#TEST

@app.route('/api/search', methods=['GET'])
def get_avg_search():
    code = request.args.get('code')
    out = request.args.get('out')
    osnm = request.args.get('osnm')

    url = 'http://www.opinet.co.kr/api/searchByName.do'
    params = {
        "code" : code,
        'out': out,
        'osnm': osnm,
    }
    response = requests.get(url, params=params)

    if response.status_code == 200:
        response_text = response.text
        search_data = json.loads(response_text)

        stations = []
        for oil in search_data['RESULT']['OIL']:
            station = {
                'name': oil['OS_NM'],
                'address': oil['NEW_ADR'],
                'GIS_X': oil['GIS_X_COOR'],
                'GIS_Y': oil['GIS_Y_COOR']
            }
            stations.append(station)

        return jsonify(stations)
    else:
        return jsonify({'error': 'Failed to fetch data from the API'}), 500






if __name__ == '__main__':
    app.run(debug=True)
