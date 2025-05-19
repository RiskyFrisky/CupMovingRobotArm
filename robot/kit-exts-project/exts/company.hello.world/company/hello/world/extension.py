import omni.ext
import omni.ui as ui
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from omni.isaac.ui.menu import MenuItemDescription
from omni.isaac.ui.element_wrappers import ScrollingWindow
import carb
import carb.events
import omni.kit.app
import omni.timeline

import gc
import omni
import omni.kit.commands
import omni.physx as _physx
import omni.timeline
import omni.ui as ui
import omni.usd
from omni.isaac.ui.element_wrappers import ScrollingWindow
from omni.isaac.ui.menu import MenuItemDescription
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from omni.usd import StageEventType

import paho.mqtt.client as mqtt
import json
import uuid
import threading

EXTENSION_TITLE = "Cup moving robot arm extension"

# Extension class
class CompanyHelloWorldExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        """
        Called when the extension is started.

        Args:
            ext_id (str): The extension ID.
        """
        print("[company.hello.world] company hello world startup")

        self.ext_id = ext_id

        # Create a UI window for the extension
        self._window = ScrollingWindow(
            title=EXTENSION_TITLE, width=300, height=300, visible=True, dockPreference=ui.DockPreference.RIGHT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        # Add a label to the window
        with self._window.frame:
            with ui.VStack():
                self.label = ui.Label("Click play button to start the client.")

        # Register the extension action
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            ext_id,
            f"CreateUIExtension:{EXTENSION_TITLE}",
            self._menu_callback,
            description=f"Add {EXTENSION_TITLE} Extension to UI toolbar",
        )
        self._menu_items = [
            MenuItemDescription(name=EXTENSION_TITLE, onclick_action=(ext_id, f"CreateUIExtension:{EXTENSION_TITLE}"))
        ]
        add_menu_items(self._menu_items, EXTENSION_TITLE)

        self.client = None
        self.desired_position_sub = None

        # Events
        self._usd_context = omni.usd.get_context()
        self._stage_event_sub = None

        self._on_window(self._window.visible)

    def on_shutdown(self):
        """
        Called when the extension is shut down.
        """
        print("[company.hello.world] company hello world shutdown")

        # Remove menu items and deregister actions
        remove_menu_items(self._menu_items, EXTENSION_TITLE)

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{EXTENSION_TITLE}")

        if self._window:
            self._window = None
        gc.collect()

    def _menu_callback(self):
        """
        Callback for menu item.
        """
        self._window.visible = not self._window.visible

    def _on_window(self, visible):
        """
        Handle window visibility changes.

        Args:
            visible (bool): Whether the window is visible.
        """
        print(f"Window visible: {visible}")
        if self._window.visible:
            # Subscribe to Stage and Timeline Events
            self._usd_context = omni.usd.get_context()
            events = self._usd_context.get_stage_event_stream()
            self._stage_event_sub = events.create_subscription_to_pop(self._on_stage_event)
        else:
            self._usd_context = None
            self._stage_event_sub = None

    def on_desired_position(self, event: carb.events.IEvent):
        """
        Handle bin trigger events.

        Args:
            event (carb.events.IEvent): The event object.
        """
        data = event.payload["joints"]

        payload = {
            "Angle0": data[0],
            "Angle1": data[1],
            "Angle2": data[2],
            "Angle3": data[3],
            "Angle4": data[4],
            "GripperOpen": data[5],
        }
        json_payload = json.dumps(payload)

        # print(f"on_desired_position = {json_payload}")
        self.client.publish("desiredPosition", json_payload, qos=2)

    def _on_stage_event(self, event):
        """
        Handle stage events.

        Args:
            event (carb.events.IEvent): The event object.
        """
        # print(f"Stage event: {event.type}")
        if event.type == int(StageEventType.OMNIGRAPH_START_PLAY):
            # NOTE: this event gets triggered when starting and resuming simulation
            print(f"OMNIGRAPH_START_PLAY: {event.type}")
            # Start the client when simulation starts
            threading.Thread(target=self.start_client, daemon=True).start()
            self.label.text = "Client starting..."
            # Subscribe to desiredPosition events
            message_bus = omni.kit.app.get_app().get_message_bus_event_stream()
            desired_position_msg = carb.events.type_from_string("desiredPosition")
            self.desired_position_sub = message_bus.create_subscription_to_pop_by_type(desired_position_msg, self.on_desired_position)
        elif event.type == int(StageEventType.OMNIGRAPH_STOP_PLAY):
            # NOTE: this event gets triggered when pausing and stopping simulation
            print(f"OMNIGRAPH_STOP_PLAY: {event.type}")
            # Stop the client when simulation stops
            self.stop_client()
            self.label.text = "Click play button to start the client."
            self.desired_position_sub = None
        # else:
        #     print(f"Unhandled event: {event.type}")

    # MARK: - MQTT Client
    def on_connect(self, client, userdata, flags, reason_code, properties):
        print("Connected with result code:", reason_code)
        client.subscribe("reportedPosition", qos=2)
        client.subscribe("objectPosition", qos=2)

        self.label.text = "Client is connected"

    def on_message(self, client, userdata, msg):
        try:
            # Try decoding the payload as JSON
            data = json.loads(msg.payload.decode())
            # print(f"Received JSON on {msg.topic}: {data}")
            if msg.topic == "reportedPosition":
                print(f"Reported position: {data}")
                message_bus = omni.kit.app.get_app().get_message_bus_event_stream()
                sim_command_msg = carb.events.type_from_string("reportedPosition")
                # send double[]
                data = [
                    data["Angle0"],
                    data["Angle1"],
                    data["Angle2"],
                    data["Angle3"],
                    data["Angle4"],
                    data["GripperOpen"],
                ]
                message_bus.push(sim_command_msg, payload={ "joints": data })
            elif msg.topic == "objectPosition":
                print(f"Object position: {data}")
                message_bus = omni.kit.app.get_app().get_message_bus_event_stream()
                sim_command_msg = carb.events.type_from_string("objectPosition")
                # send double[3]
                data = [
                    data["x"],
                    data["y"],
                    data["z"]
                ]
                message_bus.push(sim_command_msg, payload={ "position": data })
        except json.JSONDecodeError:
            # print(f"Received non-JSON message on {msg.topic}: {msg.payload.decode()}")
            pass

    def start_client(self):
        """
        Start the client.
        """
        if self.client is None:
            random_suffix = uuid.uuid4().hex[:8]  # short random string
            client_id = f"isaac_sim_ext_{random_suffix}"
            self.client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv5)
            self.client.on_connect = self.on_connect
            self.client.on_message = self.on_message

        print("Connecting to broker...")

        # Broker settings
        broker_address = "localhost"
        self.client.connect(broker_address, 1883, 60)
        self.client.loop_start()

    def stop_client(self):
        """
        Stop the client.
        """
        self.client.unsubscribe("reportedPosition")
        self.client.unsubscribe("objectPosition")
        self.client.disconnect()
        self.client.loop_stop()
        print('Client stopped')