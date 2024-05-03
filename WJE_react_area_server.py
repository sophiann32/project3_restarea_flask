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

def get_db_connection():
    dsn = cx_Oracle.makedsn("192.168.0.27", 1521, service_name="xe")
    return cx_Oracle.connect(user="restarea", password="1577", dsn=dsn)



# 5번 지은 데이터 불러오기



def get_request_search_url():
    url = 'http://www.opinet.co.kr/api/searchByName.do'
    params = {
        "code" : 'F240409104',
        'out': 'json',
        'osnm': '보라매'
    }
    response = requests.get(url, params=params)
    if response.status_code == 200:
        return response.text
    else:
        return None

@app.route('/api/search', methods=['GET','POST'])
def get_avg_search():
    # Opinet API를 사용하여 평균 가격 정보를 가져오는 함수
    response_text = get_request_search_url()

    if response_text:
        # JSON 형식의 응답을 파싱하여 필요한 정보를 추출합니다.
        search_data = json.loads(response_text)

        # Opinet API의 응답 형식에 따라 필드 값을 추출합니다.
        stations = []
        for oil in search_data['RESULT']['OIL']:
            station = {
                'name': oil['OS_NM'],
                'address': oil['NEW_ADR'],
                # 'GIS_X': oil['GIS_X_COOR'],
                # 'GIS_Y': oil['GIS_Y_COOR'],
                # 추가적으로 필요한 정보들을 추출합니다.
            }
            stations.append(station)

        return jsonify(stations)
    else:
        return jsonify({'error': 'Failed to fetch data from the API'}), 500




















# 1번 주변 주유소 데이터 뿌리기
# WGS84에서 TM128으로 변환하는 함수
def user_location_to_tm128(latitude, longitude):
    WGS84 = {'proj': 'latlong', 'datum': 'WGS84', 'ellps': 'WGS84'}
    TM128 = {
        'proj': 'tmerc', 'lat_0': '38N', 'lon_0': '128E', 'ellps': 'bessel',
        'x_0': '400000', 'y_0': '600000', 'k': '0.9999',
        'towgs84': '-146.43,507.89,681.46'
    }
    proj_WGS84 = Proj(**WGS84)
    proj_TM128 = Proj(**TM128)
    x_point, y_point = transform(proj_WGS84, proj_TM128, longitude, latitude)
    return x_point, y_point

def get_gas_stations(x, y, radius, prodcd):
    # API URL 설정
    url = "http://www.opinet.co.kr/api/aroundAll.do"
    # 요청 파라미터 설정
    params = {
        "code": "F240409104",
        "x": x,
        "y": y,
        "radius": radius,
        "sort": 1,
        "prodcd": prodcd,
        "out": "xml"
    }
    response = requests.get(url, params=params)
    # 요청 성공 시
    if response.status_code == 200:
        logger.info(f"X={x}, Y={y}에 대한 API 응답 성공")
        return response.text
    else:
        # 요청 실패 시 로그 기록
        logger.error(f"X={x}, Y={y}에 대한 API 요청 실패: 상태 코드 {response.status_code}")
        return None
def tm_to_wgs84(x, y):
    # 변환 파라미터 설정
    in_proj = Proj('+proj=tmerc +lat_0=38 +lon_0=128 +k=1 +x_0=400000 +y_0=600000 +ellps=bessel +towgs84=-146.43,507.89,681.46')
    out_proj = Proj(proj='latlong', datum='WGS84')
    # 좌표 변환 실행
    longitude, latitude = transform(in_proj, out_proj, x, y)
    logger.info(f"TM X={x}, TM Y={y} -> WGS 84 위도={latitude}, 경도={longitude}로 변환됨")
    return longitude, latitude

# 주유소 정보를 제공하는 API 엔드포인트
@app.route('/get-stations', methods=['POST'])
def get_stations():
    data = request.json  # 클라이언트로부터 JSON 데이터 수신
    radius = data.get('radius', 5000)  # 반경, 기본값은 5000m
    latitude = data['latitude']  # 위도
    longitude = data['longitude']  # 경도
    prodcd = data.get('prodcd', 'B027')  # 연료 코드, 기본값은 휘발유

    logger.info(f"POST 데이터 수신: {data}")

    # 좌표 변환 수행
    tm_x, tm_y = user_location_to_tm128(latitude, longitude)
    # 주유소 정보 요청
    gas_station_data = get_gas_stations(tm_x, tm_y, radius, prodcd)

    # 주유소 데이터 처리
    if gas_station_data:
        root = ET.fromstring(gas_station_data)
        stations_info = []
        for oil in root.findall('.//OIL'):
            station_name = oil.find('OS_NM').text
            price = oil.find('PRICE').text
            distance = oil.find('DISTANCE').text
            brand = oil.find('POLL_DIV_CO').text
            tm_x = float(oil.find('GIS_X_COOR').text)
            tm_y = float(oil.find('GIS_Y_COOR').text)
            wgs_longitude, wgs_latitude = tm_to_wgs84(tm_x, tm_y)
            stations_info.append({
                "name": station_name,
                "latitude": wgs_latitude,
                "longitude": wgs_longitude,
                "price": price,
                "distance": distance,
                "brand": brand
            })
        logger.info(f"처리된 주유소 정보: {stations_info}")
        return jsonify(stations_info)  # 처리된 주유소 정보 JSON으로 반환
    else:
        # 데이터 파싱 실패 또는 API 요청 실패 처리
        logger.error("주유소 데이터 파싱 실패 또는 API 요청 실패")
        return jsonify({"error": "주유소 정보를 가져오는데 실패하였습니다."}), 500






def get_gas_stations2(x, y):
    url = "http://www.opinet.co.kr/api/aroundAll.do"
    params = {
        "code": "F240409104",
        "x": x,
        "y": y,
        "radius": 2000,  # 반경 2km
        "sort": 1,
        "prodcd": "B027",
        "out": "xml"
    }
    response = requests.get(url, params=params)
    if response.status_code == 200:
        return response.text
    else:
        logger.error(f"API 요청 실패: {response.status_code}")
        return None


@app.route('/api/gas-stations', methods=['POST'])
def api_get_gas_stations():
    data = request.get_json()
    latitude = data['latitude']
    longitude = data['longitude']

    tm_x, tm_y = user_location_to_tm128(latitude, longitude)
    gas_station_data = get_gas_stations2(tm_x, tm_y)

    if gas_station_data:
        root = ET.fromstring(gas_station_data)
        stations = []
        for oil in root.findall('.//OIL'):
            stations.append({
                'name': oil.find('OS_NM').text,
                'price': oil.find('PRICE').text,
                'brand': oil.find('POLL_DIV_CO').text
            })
        return jsonify(stations)
    else:
        return jsonify({'error': 'Failed to fetch gas station data'}), 500




# 2번 전국 평균 오일 가격 데이터 뿌리기
def get_request_url():
    url = 'http://www.opinet.co.kr/api/avgAllPrice.do'
    params = {
        "code":'F240409104',
        'out': 'json'
    }
    response = requests.get(url, params=params)
    if response.status_code == 200:
        return response.text
    else:
        return None

@app.route('/api/avgAllPrice', methods=['GET'])
def get_avg_all_price():
    # Opinet API를 사용하여 평균 가격 정보를 가져오는 함수
    response_text = get_request_url()

    if response_text:
        # JSON 형식의 응답을 파싱하여 필요한 정보를 추출합니다.
        avg_price_data = json.loads(response_text)

        # Opinet API의 응답 형식에 따라 필드 값을 추출합니다.
        stations = []
        for oil in avg_price_data['RESULT']['OIL']:
            station = {
                'date': oil['TRADE_DT'],
                'name': oil['PRODNM'],
                'price': oil['PRICE'],
                'diff': oil['DIFF'],
                # 추가적으로 필요한 정보들을 추출합니다.
            }
            stations.append(station)

        return jsonify(stations)
    else:
        return jsonify({'error': 'Failed to fetch data from the API'}), 500







# 3번 최근 오일 가격 데이터 뿌리기
def get_url_avg():
    url = 'https://www.opinet.co.kr/api/avgRecentPrice.do'
    params = {
        "code":'F240409104',
        'out': 'json'
    }
    response = requests.get(url, params=params)
    if response.status_code == 200:
        return response.text
    else:
        return None

@app.route('/api/avgRecentPrice', methods=['GET'])
def get_avg_recent_price():
    # Opinet API를 사용하여 평균 가격 정보를 가져오는 함수
    response_text = get_url_avg()

    if response_text:
        # JSON 형식의 응답을 파싱하여 필요한 정보를 추출합니다.
        avg_price_data = json.loads(response_text)

        # Opinet API의 응답 형식에 따라 필드 값을 추출합니다.
        oil_data = {}

        for oil in avg_price_data['RESULT']['OIL']:
            date = oil['DATE']
            oil_type = oil['PRODCD']
            price = oil['PRICE']

            # oil_type을 키로 사용하여 데이터를 그룹화합니다.
            if oil_type not in oil_data:
                oil_data[oil_type] = []

            # 각 오일 타입에 대한 날짜와 가격을 따로 저장합니다.
            oil_data[oil_type].append({'date': date, 'price': price})

        # 각 오일 타입에 대한 데이터를 JSON 형태로 반환합니다.
        return jsonify(oil_data)

    else:
        return jsonify({'error': 'Failed to fetch data from the API'}), 500

# 4번 도넛 전기차 위치 정보 뿌리기
@app.route('/location', methods=['POST'])
def handle_location():
    data = request.get_json()
    latitude = data['latitude']
    longitude = data['longitude']
    # 데이터베이스 쿼리 실행
    result = query_database(latitude, longitude)
    return jsonify(result)


def query_database(latitude, longitude):
    try:
        connection = get_db_connection()
        cursor = connection.cursor()

        # 쿼리 파라미터 바인딩
        cursor.execute("""   SELECT *
            FROM (SELECT statNm,distance
                  FROM (SELECT statNm,

                               (6371 * acos(cos(:inputLatitude * (acos(-1) / 180)) * cos(lat * (acos(-1) / 180)) *
                                            cos((lng - :inputLongitude) * (acos(-1) / 180)) +
                                            sin(:inputLatitude * (acos(-1) / 180)) *
                                            sin(lat * (acos(-1) / 180)))) AS distance,
                               ROW_NUMBER()                                  OVER (PARTITION BY statNm, statId ORDER BY (6371 * acos(cos(:inputLatitude * (acos(-1)/180)) * cos(lat * (acos(-1)/180)) * cos((lng - :inputLongitude) * (acos(-1)/180)) + sin(:inputLatitude * (acos(-1)/180)) * sin(lat * (acos(-1)/180)))) ASC) AS rn
                        FROM chargingstations
                        WHERE lat BETWEEN :minLat AND :maxLat
                          AND lng BETWEEN :minLng AND :maxLng
                          AND (6371 * acos(cos(:inputLatitude * (acos(-1) / 180)) * cos(lat * (acos(-1) / 180)) *
                                           cos((lng - :inputLongitude) * (acos(-1) / 180)) +
                                           sin(:inputLatitude * (acos(-1) / 180)) * sin(lat * (acos(-1) / 180)))) < 1)
                  WHERE rn = 1)
            WHERE ROWNUM <= 10
    """,
                       inputLatitude=latitude, inputLongitude=longitude,
                       minLat=latitude - 0.1, maxLat=latitude + 0.1,
                       minLng=longitude - 0.1, maxLng=longitude + 0.1)

        stations = []
        for row in cursor.fetchall():
            stations.append({"Station Name": row[0], "Distance": row[1]})
            print("Station Name: ", row[0], "Distance: ", row[1])  # 콘솔에 충전소 이름과 거리 출력

        cursor.close()
        connection.close()
        return {"stations": stations}

    except cx_Oracle.DatabaseError as e:
        error, = e.args
        print(f"Database Error: {error.message}")  # 데이터베이스 오류 출력
        return {"error": str(error.message)}



API_KEY =  "7423400608"

@app.route('/restareas', methods=['GET'])
def get_restareas():
    route_name = request.args.get('route')
    if not route_name:
        return jsonify({'error': 'No route provided'}), 400

    try:
        connection = get_db_connection()
        cursor = connection.cursor()

        cursor.execute("""
            SELECT 휴게소명, 도로노선명, 위도, 경도, 휴게소전화번호, 경정비가능여부, 주유소유무, LPG충전소유무, 쉼터유무
            FROM restareas
            WHERE 도로노선명 = :route
        """, route=route_name)
        app.logger.debug("Query executed")

        columns = [col[0] for col in cursor.description]
        result = [dict(zip(columns, row)) for row in cursor.fetchall()]
        cursor.close()
        connection.close()
        app.logger.debug("Database connection closed")

        return jsonify(result)
    except Exception as e:
        app.logger.error(f"An error occurred: {e}")
        return jsonify({'error': 'Database error'}), 500

# 추가된 외부 API 호출 함수
@app.route('/restbrands', methods=['GET'])
def get_rest_brands():
    route_nm = request.args.get('routeNm')
    if not route_nm:
        return jsonify({'error': '라우트 이름이 일치하지않음'}), 400

    url = f"https://data.ex.co.kr/openapi/restinfo/restBrandList?key={API_KEY}&type=json&numOfRows=10&pageNo=1&routeNm={route_nm}"
    response = requests.get(url)
    if response.status_code == 200:
        return jsonify(response.json())
    else:
        return jsonify({'error': '데이터 전송 오류'}), 500

@app.route('/bestfoods', methods=['GET'])
def get_best_foods():
    route_nm = request.args.get('routeNm')
    if not route_nm:
        return jsonify({'error': '라우트 이름이 일치하지않음'}), 400

    url = f"https://data.ex.co.kr/openapi/restinfo/restBestfoodList?key={API_KEY}&type=json&numOfRows=10&pageNo=1&routeNm={route_nm}"
    response = requests.get(url)
    if response.status_code == 200:
        return jsonify(response.json())
    else:
        return jsonify({'error': '데이터 전송 오류'}), 500

@app.route('/facilities', methods=['GET'])
def get_facilities():
    route_nm = request.args.get('routeNm')
    if not route_nm:
        return jsonify({'error': '라우트 이름이 일치하지않음'}), 400

    url = f"https://data.ex.co.kr/openapi/business/serviceAreaRoute?key={API_KEY}&type=json&numOfRows=10&pageNo=1&routeName={route_nm}"
    response = requests.get(url)
    if response.status_code == 200:
        return jsonify(response.json())
    else:
        return jsonify({'error': '데이터 전송 오류'}), 500

@app.route('/fuelprices', methods=['GET'])
def get_fuel_prices():
    route_nm = request.args.get('routeNm')
    if not route_nm:
        return jsonify({'error': '라우트 이름이 일치하지않음'}), 400

    url = f"https://data.ex.co.kr/openapi/business/curStateStation?key={API_KEY}&type=json&numOfRows=10&pageNo=1&routeName={route_nm}"
    response = requests.get(url)
    if response.status_code == 200:
        return jsonify(response.json())
    else:
        return jsonify({'error': '데이터 전송 오류'}), 500

def user_location_to_tm128(latitude, longitude):
    transformer = Transformer.from_crs("EPSG:4326", "+proj=tmerc +lat_0=38N +lon_0=128E +ellps=bessel +x_0=400000 +y_0=600000 +k=0.9999 +towgs84=-146.43,507.89,681.46", always_xy=True)
    x_point, y_point = transformer.transform(longitude, latitude)
    print(f"Transformed Latitude and Longitude ({latitude}, {longitude}) to TM128 coordinates ({x_point}, {y_point})")
    return x_point, y_point

# 주유소 정보를 가져오는 함수
def get_gas_stations22(x, y):
    url = "http://www.opinet.co.kr/api/aroundAll.do"
    params = {"code": "F240409104", "x": x, "y": y, "radius": 5000, "sort": 1, "prodcd": "B027", "out": "xml"}
    response = requests.get(url, params=params)
    if response.status_code == 200:
        print(f"Received response from API for coordinates ({x}, {y}) with status {response.status_code}")
        return response.text
    else:
        print(f"Failed to fetch data for coordinates ({x}, {y}) with status {response.status_code}")
        response.raise_for_status()


# 주유소 정보를 제공하는 API 라우트
@app.route('/get_gas_stations22', methods=['post'])
def get_gas_stations_route():
    try:
        data = request.json
        latitude = data.get('latitude')
        longitude = data.get('longitude')
        if not latitude or not longitude:
            return jsonify({"error": "Missing latitude or longitude"}), 400
        print(f"Received request for latitude {latitude} and longitude {longitude}")
        tm_x, tm_y = user_location_to_tm128(latitude, longitude)
        gas_station_data = get_gas_stations(tm_x, tm_y)
        print('주유소데이터'+ gas_station_data)
        return jsonify({"data": gas_station_data})
    except ValueError as e:
        print(f"Error: {str(e)}")
        return jsonify({"error": str(e)}), 400
    except Exception as e:
        print(f"Unexpected Error: {str(e)}")
        return jsonify({"error": str(e)}), 500



# -----------------------------------전기차충전소코드-------------------------------------------


@app.route('/api/charging-stations-jeju')
def get_charging_stations():
    conn = get_db_connection()
    cursor = conn.cursor()

    api_url = "http://api.jejuits.go.kr/api/infoEvList?code=860634"
    response = requests.get(api_url)
    api_data = response.json()

    api_info = {item['id']: {'fast': item['fast'], 'slow': item['slow']} for item in api_data['info']}
    api_ids = list(api_info.keys())

    def chunked_list(lst, n):
        for i in range(0, len(lst), n):
            yield lst[i:i + n]

    results = []
    for ids_chunk in chunked_list(api_ids, 1000):
        id_str = ', '.join(f"'{id}'" for id in ids_chunk)
        query = f"""
        SELECT id, addr, use_time, free, b_call, type, x_crdn, y_crdn
        FROM charging_info
        WHERE id IN ({id_str})
        """
        cursor.execute(query)
        for row in cursor:
            result = {
                "ID": row[0],
                "Address": row[1],
                "Usage Time": row[2],
                "Free": row[3],
                "Booking Call": row[4],
                "Type": row[5],
                "X Coordinate": row[6],
                "Y Coordinate": row[7],
                "Fast Chargers": api_info[row[0]]['fast'],
                "Slow Chargers": api_info[row[0]]['slow']
            }
            results.append(result)

    cursor.close()
    conn.close()
    return jsonify(results)

@app.route('/api/tourism-spots')
def get_tourism_spots():
    conn = get_db_connection()
    cursor = conn.cursor()

    query = """
    SELECT CONTENTS_ID, CONTENTS_LABEL, TITLE, ADDRESS, ROAD_ADDRESS, TAG, INTRODUCTION, LATITUDE, LONGITUDE, POST_CODE, PHONE_NO, IMG_PATH, THUMBNAIL_PATH
    FROM tourism_spots
    """
    cursor.execute(query)
    columns = [col[0] for col in cursor.description]
    results = [dict(zip(columns, row)) for row in cursor.fetchall()]

    cursor.close()
    conn.close()
    return jsonify(results)








if __name__ == '__main__':
    app.run(debug=True)