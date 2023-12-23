'''Implements simple mqtt sub function to receive a single message
with a timeout.

Adapted from https://github.com/eclipse/paho.mqtt.python/issues/655
'''

from typing import Optional, Any
import threading
from paho.mqtt.client import Client
from paho.mqtt.publish import single as pub_single


def sub_single(
    topic: str,
    hostname: str,
    timeout: Optional[float] = None,
    **mqtt_kwargs,
):
    """
    Modeled closely after the paho version, this also includes some try/excepts and
    a timeout. Note that this _does_ disconnect after receiving a single message.
    """

    lock: Optional[threading.Lock]

    def on_connect(client: Client, userdata, flags, rc):
        client.subscribe(userdata["topics"])
        return

    def on_message(client: Client, userdata, message):

        userdata["messages"] = message
        client.disconnect()

        if userdata["lock"]:
            userdata["lock"].release()

        return

    if timeout:
        lock = threading.Lock()
    else:
        lock = None

    userdata: dict[str, Any] = {
        "topics": [(topic, 0)],
        "messages": None,
        "lock": lock,
    }

    client = Client(userdata=userdata)
    client.username_pw_set(**mqtt_kwargs['auth'])
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(hostname)

    if timeout is None:
        client.loop_forever()
    else:
        assert lock is not None
        lock.acquire()
        client.loop_start()
        lock.acquire(timeout=timeout)
        client.loop_stop()
        client.disconnect()

    return userdata["messages"]

# pub_single('homeassistant/pippinodock/relay/cmd', client_id='autodock_server', payload=0, auth=MQTT_AUTH, hostname=MQTT_BROKER)
# mqtt_msg = sub_single('homeassistant/pippinodock/current/state', client_id='autodock_server', auth=MQTT_AUTH, hostname=MQTT_BROKER, timeout=5)
# print(mqtt_msg.payload)