import argparse
import asyncio
import base64
import csv
import json
import logging
import os
import struct

import aiohttp
import paho.mqtt.client as mqtt
from aiohttp import web

# TODO
# * Reread data-file on start to fill seen/missed
# * Determine what to do with double readings (same coord)
# * Generate an empty datafile (with headers) if it does not already exist


parser = argparse.ArgumentParser()
parser.add_argument(
    '-d', '--debug', action="store_const", dest="loglevel", const=logging.DEBUG,
    default=logging.WARNING,
)
parser.add_argument(
    '-v', '--verbose', action="store_const", dest="loglevel", const=logging.INFO,
)
parser.add_argument(
    '-p', '--port', type=int, default=8080,
)
parser.add_argument(
    '-H', '--host', default='0.0.0.0',
)

clients = set()
seen = list()
missed = list()

FIELDS = [
    "latitude", "longitude", "seen", "received",
    "lsnr", "rssi", "datarate",
    "gateway", "gateway_latitude", "gateway_longitude",
]
HOME_DIR = os.path.dirname(os.path.realpath(__file__))
HTML_DIR = os.path.join(HOME_DIR, 'html')
DATA_FILE = os.path.join(HOME_DIR, 'data.csv')


# noinspection PyUnusedLocal
def on_connect(client, userdata, flags, rc):
    logging.info('Connected to host, subscribing to uplink messages')
    client.subscribe('+/devices/+/up')


# noinspection PyUnusedLocal
def on_message(client, userdata, msg):
    logging.debug('Received message {}'.format(str(msg.payload)))

    try:
        message = json.loads(msg.payload.decode('utf8'))
        payload = base64.b64decode(message.get('payload', ''))
        process_data(message, payload)

    # python2 uses ValueError and perhaps others, python3 uses JSONDecodeError
    except Exception as e:
        logging.warning('Error parsing JSON or processing payload')
        logging.warning(e)


def process_data(message, payload):
    if message["port"] != 10:
        logging.info('Ignoring message on port {}'.format(message['port']))
        return

    if len(payload) % 7 != 0:
        logging.warning('Invalid rangetest packet received with length {}'.format(len(payload)))
        return

    coords = []

    for offset in range(0, len(payload), 7):
        dr = payload[offset]
        lat = unpack_coord(payload[offset + 1:offset + 4])
        lon = unpack_coord(payload[offset + 4:offset + 7])
        coords.append((dr, lat, lon))

    metadata = message.get('metadata')

    if coords[0] not in seen:
        seen.append(coords[0])
        emit_coord(coords[0], metadata, True)

    if coords[0] in missed:
        missed.remove(coords[0])

    for coord in coords[1:]:
        if coord not in seen and coord not in missed:
            missed.append(missed)
            emit_coord(coord, metadata, False)

def int_to_dr(num):
	if num == 0:
		return 'SF12BW125'
	elif num == 1:
		return 'SF11BW125'
	elif num == 2:
		return 'SF10BW125'
	elif num == 3:
		return 'SF9BW125'
	elif num == 4:
		return 'SF8BW125'
	elif num == 5:
		return 'SF7BW125'
	elif num == 6:
		return 'SF7BW250'
	else:
		return '???'

def emit_coord(coord, metadata, seen):
    for m in metadata:
        if seen:
            emit_line(
                latitude=coord[1], longitude=coord[2],
                seen=seen, received=m.get('gateway_timestamp'),
                lsnr=m.get('lsnr'), datarate=m.get('datarate'), rssi=m.get('rssi'),
                gateway=m.get('gateway_eui'),
                gateway_latitude=m.get('latitude'), gateway_longitude=m.get('longitude'),
            )
        else:
            dr = int_to_dr(coord[0])
            emit_line(
                latitude=coord[1], longitude=coord[2],
                seen=seen, received=m.get('gateway_timestamp'),
                lsnr=0, datarate=dr, rssi=0,
                gateway=m.get('gateway_eui'),
                gateway_latitude=m.get('latitude'), gateway_longitude=m.get('longitude'),
            )



def emit_line(**kwargs):
    with open(DATA_FILE, 'a') as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writerow(kwargs)

    for client in clients:
        client.send_str(json.dumps([kwargs]))


def unpack_coord(data):
    # Latitude/Longitude are each packed as 3-byte fixed point with a
    # divisor of 32768.
    return struct.unpack('>i', b'\x00' + data[:3])[0] / 32768.0


@asyncio.coroutine
def check_mqtt(client):
    while True:
        client.loop_misc()
        yield from asyncio.sleep(10)


# noinspection PyUnusedLocal
@asyncio.coroutine
def home(request):
    with open(os.path.join(HTML_DIR, 'index.html'), 'r') as f:
        content = f.read()
    return web.Response(body=content.encode('utf8'))


@asyncio.coroutine
def websocket(request):
    ws = web.WebSocketResponse()
    yield from ws.prepare(request)

    clients.add(ws)
    with open(DATA_FILE, 'r') as f:
        reader = csv.DictReader(f)
        ws.send_str(json.dumps([l for l in reader]))

    running = True
    while running:
        msg = yield from ws.receive()
        if msg.tp == aiohttp.MsgType.close:
            running = False
        elif msg.tp == aiohttp.MsgType.text:
            print(msg.data)

    clients.remove(ws)

    return ws


def get_mqtt_client(app_eui=None, access_key=None, ca_cert_path=None, host=None):
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    port = 1883

    if app_eui is not None and access_key is not None:
        client.username_pw_set(app_eui, password=access_key)
    else:
        logging.warning('No App EUI or Access key set')

    if ca_cert_path:
        if not os.path.exists(ca_cert_path):
            logging.warning(
                'CA Certificate path specified does not exist, falling back to non-TLS')
        else:
            client.tls_set(ca_cert_path)
            port = 8883

    logging.info('Connecting to {} on port {}'.format(host, port))
    client.connect(host, port=port)
    client.loop_start()
    return client


def main():
    app_eui = os.environ.get('TTN_APP_EUI')
    access_key = os.environ.get('TTN_ACCESS_KEY')
    ttn_host = os.environ.get('TTN_HOST', 'staging.thethingsnetwork.org')
    ca_cert_path = os.environ.get('TTN_CA_CERT_PATH', 'mqtt-ca.pem')

    args = parser.parse_args()
    logging.basicConfig(level=args.loglevel)

    # Setup the MQTT client
    client = get_mqtt_client(
        app_eui=app_eui,
        access_key=access_key,
        host=ttn_host,
        ca_cert_path=ca_cert_path
    )

    # Configure the web application
    app = web.Application()
    app.router.add_route(
        'GET', '/', home,
    )
    app.router.add_route(
        'GET', '/data/', websocket,
    )
    app.router.add_static(
        '/static/',
        os.path.join(os.path.dirname(os.path.realpath(__file__)), 'html'),
    )

    # Start the periodical MQTT update and the web app
    asyncio.async(check_mqtt(client))
    web.run_app(app, port=args.port, host=args.host)


if __name__ == "__main__":
    main()
