# Copyright (c) 2024 ADVANTICS SAS
# Original author: Axel Voitier
# Part of Advantics examples
# MIT licensed
#
# spell-checker:words
# spell-checker:ignore EVSE EVCC exctype excinst exctb incl
from __future__ import annotations

# System imports
import struct
import time
from enum import IntEnum
from pathlib import Path
from threading import Event, Thread
from typing import TYPE_CHECKING

# Third-party imports
import can
import typer
from rich import print

if TYPE_CHECKING:
    from collections.abc import Callable
    from types import TracebackType
    from typing import Any, Self


class FrameID(IntEnum):
    """Enumeration of all CAN messages of the interface and their frame IDs"""

    # Controller sends
    EVSE_Information = 0x600
    AC_Control = 0x601
    DC_Control = 0x602
    CCS_Extra_Information = 0x603
    ADM_CS_EVCC_Inputs = 0x604

    # BMS / vehicle side sends
    EV_Information = 0x610
    AC_Status = 0x611
    DC_Status1 = 0x612
    DC_Status2 = 0x613
    EV_Energy_Request = 0x614
    EV_V2X_Energy_Request = 0x615
    EV_Extra_BPT_Information = 0x616


class CommunicationStage(IntEnum):
    """Enumeration of communication stages.
    Used in message [0x600] EVSE_Information.
    """

    Initialising = 0
    Waiting_For_EVSE = 1
    Negotiating_Connection = 2
    Connected_With_Full_Info = 3
    Insulation_Test = 4
    Precharge = 5
    Waiting_For_Charge = 6
    Charging = 7
    Ending_Charge = 8
    Welding_Detection = 9
    Closing_Communication = 10

    def __str__(self) -> str:
        return self.name


class ChargingProtocol(IntEnum):
    """Enumeration of charging protocols.
    Used in message [0x600] EVSE_Information.
    """

    NONE = 0
    CCS_PWM = 1
    CCS_DIN_70121_2012_v2 = 2
    CCS_ISO_15118_2013_v2 = 3
    CCS_ISO_15118_2022 = 4


class InletPins(IntEnum):
    """Enumeration of inlet pins.
    Used in message [0x600] EVSE_Information.
    """

    NONE = 0
    CCS_AC = 1
    CCS_AC_Single_Phase_Core = 2
    CCS_AC_Three_Phase_Core = 3
    CCS_DC_Core = 4
    CCS_DC_Extended = 5


class Simulator(can.Listener):
    """Simulator of BMS/vehicle side compatible with Advantics PEV Generic CAN interface v2"""

    # Internal states definitions
    # Session info
    session_stage: CommunicationStage
    session_protocol: ChargingProtocol
    session_pins: InletPins
    # EVES info
    evse_max_current: float
    evse_ac_ready: bool
    # EV info
    ev_soc: int
    ev_energy_capacity: float
    ev_ac_ready: bool
    ev_dc_battery_voltage: float
    ev_dc_inlet_voltage: float
    ev_dc_current_request: float
    ev_dc_present_current: float
    ev_dc_contactors_closed: bool
    ev_dc_request_normal_stop: bool
    # Commands from controller
    command_close_contactors: bool

    def __init__(
        self,
        app: Application,
        charger_dead_time: float = 1,
        charger_voltage_ramp_up_slope: float = 200,
        contactors_delay: float = 0.6,
    ) -> None:
        self._app = app
        self._bus = app.bus

        # Simulation parameters
        self._charger_dead_time = charger_dead_time
        self._charger_voltage_ramp_up_slope = charger_voltage_ramp_up_slope
        self._contactors_delay = contactors_delay

        # Initialise internal states
        self._slope_start_voltage = 0
        self.session_stage = CommunicationStage.Initialising
        self._last_stage = CommunicationStage.Initialising
        self.session_protocol = ChargingProtocol.NONE
        self.session_pins = InletPins.NONE
        self.evse_max_current = 0
        self.evse_ac_ready = False
        self.command_close_contactors = False
        self._previous_contactors_command = False
        self.reset()

        # Periodic messages
        # EV_Information
        self._ev_information_msg = can.Message(
            arbitration_id=FrameID.EV_Information,
            is_extended_id=False,
            data=self.encode_ev_information(),
        )
        self._ev_information_task = self._bus.send_periodic(
            self._ev_information_msg,
            period=0.1,
            modifier_callback=self._send_callback_ev_information,
        )
        # DC_Status1
        self._dc_status1_msg = can.Message(
            arbitration_id=FrameID.DC_Status1,
            is_extended_id=False,
            data=self.encode_dc_status1(),
        )
        self._dc_status1_task = self._bus.send_periodic(
            self._dc_status1_msg,
            period=0.1,
            modifier_callback=self._send_callback_dc_status1,
        )
        # DC_Status2
        self._dc_status2_msg = can.Message(
            arbitration_id=FrameID.DC_Status2,
            is_extended_id=False,
            data=self.encode_dc_status2(),
        )
        self._dc_status2_task = self._bus.send_periodic(
            self._dc_status2_msg,
            period=0.1,
            modifier_callback=self._send_callback_dc_status2,
        )

    # Internal states

    def reset(self) -> None:
        """Reset our internal states"""
        self.ev_soc = 30
        self.ev_energy_capacity = 75.525
        self.ev_ac_ready = False
        self.ev_dc_battery_voltage = 330
        self.ev_dc_inlet_voltage = 0
        self.ev_dc_current_request = 200
        self.ev_dc_present_current = 0
        self.ev_dc_contactors_closed = False
        self.ev_dc_request_normal_stop = False

    # Messages reading and state processing

    def on_message_received(self, msg: can.Message) -> None:
        """Main callback for every CAN message received.

        Basic principle is we look for message that interest us. When there is one,
        we decode it (NB: the decode functions directly change our internal state variables).
        """

        # Messages from the controller
        if msg.arbitration_id == FrameID.EVSE_Information:
            self.decode_evse_information(msg.data)
            if self._last_stage != self.session_stage:
                self.update_state()
                self._last_stage = self.session_stage

        elif msg.arbitration_id == FrameID.AC_Control:
            self.decode_ac_control(msg.data)

        elif msg.arbitration_id == FrameID.DC_Control:
            self.decode_dc_control(msg.data)
            self.handle_dc_control()  # DC_Control contains commands

        # elif msg.arbitration_id == FrameID.CCS_Extra_Information:
        #     self.decode_ccs_extra_information(msg.data)

        # elif msg.arbitration_id == FrameID.ADM_CS_EVCC_Inputs:
        #     self.decode_adm_cs_evcc_inputs(msg.data)

        # Messages to the controller
        # We listen for those only to update our own internal states in case
        # these are sent externally (eg. you playing around with sending some CAN messages)
        # You do not have to do the same in your implementation.

        elif msg.arbitration_id == FrameID.EV_Information:
            self.decode_ev_information(msg.data)

        elif msg.arbitration_id == FrameID.AC_Status:
            self.decode_ac_status(msg.data)

        elif msg.arbitration_id == FrameID.DC_Status1:
            self.decode_dc_status1(msg.data)

        elif msg.arbitration_id == FrameID.DC_Status2:
            self.decode_dc_status2(msg.data)

        # elif msg.arbitration_id == FrameID.EV_Energy_Request:
        #     self.decode_ev_energy_request(msg.data)

        # elif msg.arbitration_id == FrameID.EV_V2X_Energy_Request:
        #     self.decode_ev_v2x_energy_request(msg.data)

        # elif msg.arbitration_id == FrameID.EV_Extra_BPT_Information:
        #     self.decode_ev_extra_bpt_information(msg.data)

    def update_state(self) -> None:
        match self.session_stage:
            case CommunicationStage.Waiting_For_EVSE:
                # This indicate we terminated a charge session (or controller just started).
                # Use it to reset our internal states.
                self.reset()

            case CommunicationStage.Precharge:
                print(f'Inlet voltage ramping up to {self.ev_dc_battery_voltage:.1f} V...')
                self._slope_start_voltage = self.ev_dc_inlet_voltage
                total_time = self._charger_dead_time + (
                    (self.ev_dc_battery_voltage - self.ev_dc_inlet_voltage)
                    / self._charger_voltage_ramp_up_slope
                )
                subdivide_dt(self.simulate_precharge, total_time, 0.1)

            case CommunicationStage.Welding_Detection:
                print('Setting present_current to 0 A')
                self.ev_dc_present_current = 0

            case _:
                pass

    def simulate_precharge(self, elapsed: float, done: bool) -> None:  # noqa: FBT001
        if elapsed <= self._charger_dead_time:
            return

        if not done:
            elapsed -= self._charger_dead_time
            self.ev_dc_inlet_voltage = min(
                self._slope_start_voltage + (self._charger_voltage_ramp_up_slope * elapsed),
                self.ev_dc_battery_voltage,
            )
        else:
            self.ev_dc_inlet_voltage = self.ev_dc_battery_voltage
            print(f'Inlet voltage at {self.ev_dc_inlet_voltage:.1f} V')

    def handle_dc_control(self) -> None:
        if self.command_close_contactors != self._previous_contactors_command:
            if self.command_close_contactors:
                print('Closing contactors...')
                call_later(self.simulate_close_contactors, self._contactors_delay)
            else:
                print('Opening contactors')
                call_later(self.simulate_open_contactors, self._contactors_delay)

            self._previous_contactors_command = self.command_close_contactors

    def simulate_close_contactors(self) -> None:
        self.ev_dc_contactors_closed = True
        self.ev_dc_inlet_voltage = self.ev_dc_battery_voltage
        print('Contactors closed')

    def simulate_open_contactors(self) -> None:
        self.ev_dc_contactors_closed = False
        self.ev_dc_inlet_voltage = 0
        print('Contactors opened')

    # Message encoding, decoding, and sending methods

    def _cap(self, value: float, min_value: float, max_value: float) -> float:
        """Cap a given value to a minimum and maximum"""
        return min(max(value, min_value), max_value)

    # EVSE_Information

    def decode_evse_information(self, data: bytes | bytearray) -> None:
        stage, protocol, pins, max_current, _ = struct.unpack('<BBBhB', data)

        self.session_stage = CommunicationStage(stage)
        self.session_protocol = ChargingProtocol(protocol)
        self.session_pins = InletPins(pins)
        self.evse_max_current = max_current / 10

    # EV_Information

    def encode_ev_information(self) -> bytes:
        return struct.pack(
            '<BH',
            self._cap(self.ev_soc, 0, 100),
            self._cap(round(self.ev_energy_capacity * 100), 0, 65535),
        )

    def decode_ev_information(self, data: bytes | bytearray) -> None:
        soc, energy_capacity = struct.unpack('<BH', data)

        self.ev_soc = soc
        self.ev_energy_capacity = energy_capacity / 100

    def _send_callback_ev_information(self, msg: can.Message) -> None:
        """Callback provided to self._bus.send_periodic() for automatic update
        of message content"""
        msg.data = self.encode_ev_information()

    # AC_Control

    def decode_ac_control(self, data: bytes | bytearray) -> None:
        (flags,) = struct.unpack('<B', data)

        self.evse_ac_ready = bool(flags & 0x01)

    # AC_Status

    def encode_ac_status(self) -> bytes:
        return struct.pack('<B', self.ev_ac_ready)

    def decode_ac_status(self, data: bytes | bytearray) -> None:
        (flags,) = struct.unpack('<B', data)

        self.ev_ac_ready = bool(flags & 0x01)

    def _send_callback_ac_status(self, msg: can.Message) -> None:
        """Callback provided to self._bus.send_periodic() for automatic update
        of message content"""
        msg.data = self.encode_ac_status()

    # DC_Control

    def decode_dc_control(self, data: bytes | bytearray) -> None:
        (flags,) = struct.unpack('<B', data)

        self.command_close_contactors = bool(flags & 0x01)

    # DC_Status1

    def encode_dc_status1(self) -> bytes:
        return struct.pack(
            '<hhhh',
            self._cap(round(self.ev_dc_current_request * 10), -32768, 32767),
            self._cap(round(self.ev_dc_present_current * 10), -32768, 32767),
            0,
            0,
        )

    def decode_dc_status1(self, data: bytes | bytearray) -> None:
        current_request, present_current, _, _ = struct.unpack('<hhhh', data)

        self.ev_dc_current_request = current_request / 10
        self.ev_dc_present_current = present_current / 10

    def _send_callback_dc_status1(self, msg: can.Message) -> None:
        """Callback provided to self._bus.send_periodic() for automatic update
        of message content"""
        msg.data = self.encode_dc_status1()

    # DC_Status2

    def encode_dc_status2(self) -> bytes:
        flags = 0
        flags |= self.ev_dc_contactors_closed << 0
        flags |= self.ev_dc_request_normal_stop << 1
        return struct.pack(
            '<BHH',
            flags,
            self._cap(round(self.ev_dc_battery_voltage * 10), 0, 65535),
            self._cap(round(self.ev_dc_inlet_voltage * 10), 0, 65535),
        )

    def decode_dc_status2(self, data: bytes | bytearray) -> None:
        flags, battery_voltage, inlet_voltage = struct.unpack('<BHH', data)

        self.ev_dc_contactors_closed = bool(flags & 0x01)
        self.ev_dc_request_normal_stop = bool(flags & 0x02)
        self.ev_dc_battery_voltage = battery_voltage / 10
        self.ev_dc_inlet_voltage = inlet_voltage / 10

    def _send_callback_dc_status2(self, msg: can.Message) -> None:
        """Callback provided to self._bus.send_periodic() for automatic update
        of message content"""
        msg.data = self.encode_dc_status2()


# Wrappers around threads to implement simulated behaviours


def call_later(callback: Callable[[], None], dt: float) -> Thread:
    """Simple delayed execution of code"""

    def wrapper() -> None:
        try:
            time.sleep(dt)
            callback()
        except Exception as ex:  # noqa: BLE001
            print(f'Got exception in call_later thread for {callback}.', str(ex))

    thread = Thread(target=wrapper)
    thread.start()

    return thread


def subdivide_dt(callback: Callable[[float, bool], None], dt: float, sub_dt: float) -> Thread:
    """Calls a callback repeatedly, separated by time sub_dt, for a maximum total time of dt.
    The callback has to take two arguments:
    - A float (elapsed): This will be the time elapsed since the beginning.
    - A boolean (done): Telling if the total time has elapsed (ie. True for the last call)

    Guaranteed to have at least one call with done=False (unless dt=0).
    Guaranteed to always have one call with done=True, and no subsequent call afterwards.
    (Unless exception in callback)
    """

    def wrapper() -> None:
        try:
            started = time.monotonic()
            now = started
            ends_at = started + dt
            while now < ends_at:
                time.sleep(max(min(sub_dt, ends_at - now), 0))
                now = time.monotonic()
                callback(now - started, False)  # noqa: FBT003
            callback(now - started, True)  # noqa: FBT003
        except Exception as ex:  # noqa: BLE001
            print(f'Got exception in subdivide_dt thread for {callback}.', str(ex))

    thread = Thread(target=wrapper)
    thread.start()

    return thread


class Application:
    """Main application class. Handles creation of various objects, and life cycle of it."""

    def __init__(self, bus_config: can.typechecking.BusConfig, **interface_config: Any) -> None:
        self._bus_config = bus_config
        self._interface_config = interface_config

        # Instantiation delayed on getter access
        self._bus: can.BusABC | None = None
        self._notifier: can.Notifier | None = None
        self._simulator: Simulator | None = None

        self.stop_event = Event()

    @property
    def bus(self) -> can.BusABC:
        """A configured CAN bus object"""
        if (bus := self._bus) is None:
            bus = self._bus = can.Bus(**self._bus_config)

        return bus

    @property
    def simulator(self) -> Simulator:
        """Simulator instance"""
        if (simulator := self._simulator) is None:
            simulator = self._simulator = Simulator(self, **self._interface_config)

        return simulator

    @property
    def notifier(self) -> can.Notifier:
        """CAN notifier instance (ie. the main reading loop)"""
        if (notifier := self._notifier) is None:
            notifier = self._notifier = can.Notifier(self.bus, [self.simulator])

        return notifier

    # Application life cycle

    def __enter__(self) -> Self:
        """Enters a with-statement by starting the application and returns itself"""
        self.start()
        return self

    def start(self) -> None:
        """Instantiate bus, simulator, and notifier, and starts notifier up"""
        self.stop_event.clear()
        _ = self.notifier  # Implicitly instantiation and start-up on getter access

    ###

    def run(self) -> None:
        """Method blocking until the application stops, or is stopped by an exception.
        NB: A Ctrl+C in the terminal will also work (raises a special exception)"""
        try:
            self.stop_event.wait()
        except Exception:
            self.stop_event.set()
            raise

    ###

    def __exit__(
        self,
        exctype: type[BaseException] | None,
        excinst: BaseException | None,
        exctb: TracebackType | None,
    ) -> bool:
        """Exits a with-statement by shutting down the application (incl. closing the bus).
        Does not handle any exception."""
        self.shutdown()
        return False

    def shutdown(self) -> None:
        """Stops the application by interrupting the run() blocking method,
        stopping notifier, and closing CAN bus."""
        self.stop_event.set()

        if self._notifier is not None:
            self._notifier.stop()
            self._notifier = None

        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None


def cli_main(
    can_config: Path = Path('can.conf'),
) -> None:
    """Simulator of BMS/vehicle side compatible with Advantics PEV Generic CAN interface v2"""
    try:
        bus_config = can.util.load_config(path=can_config)
    except can.exceptions.CanInterfaceNotImplementedError as ex:
        print(f'[red]ERROR:[/] Incorrect CAN configuration. {ex}.')
        raise typer.Abort from ex

    with Application(bus_config) as app:
        app.run()


if __name__ == '__main__':
    typer.run(cli_main)
