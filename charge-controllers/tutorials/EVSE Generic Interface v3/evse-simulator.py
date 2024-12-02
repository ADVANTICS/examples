# Copyright (c) 2024 ADVANTICS SAS
# Original author: Axel Voitier
# Part of Advantics examples
# MIT licensed
#
# spell-checker:words setpoint setpoints
# spell-checker:ignore EVSE SECC OCPP exctype excinst exctb incl
from __future__ import annotations

# System imports
import enum
import struct
import time
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


class FrameID(enum.IntEnum):
    """Enumeration of all CAN messages of the interface and their base frame IDs"""

    # Controller sends
    Advantics_Controller_Status = 0x6B000
    New_Charge_Session = 0x6B001
    Charge_Status_Change = 0x6B002
    DC_Power_Control = 0x6B003
    Charge_Session_Finished = 0x6B004
    Emergency_Stop = 0x6B005
    EV_Information_Battery = 0x6B100
    EV_Information_Voltages = 0x6B101
    EV_Information_Charge_Limits = 0x6B102
    EV_Information_Discharge_Limits = 0x6B103
    ADM_CO_CUI1_Inputs = 0x6B200
    ADM_CS_SECC_Inputs = 0x6B201
    OCPP_Control = 0x6B300

    # Power modules send
    Power_Modules_Status = 0x63000
    DC_Power_Parameters = 0x63001
    Sequence_Control = 0x63002
    ADM_CS_SECC_Outputs = 0x63201

    def for_pistol(self, index: int) -> int:
        """Adds the pistol index to the frame ID"""
        return self | (index << 24)


class ControllerState(enum.IntEnum):
    """Enumeration of controller states.
    Used in message [0x6B000] Advantics_Controller_Status.
    """

    Initialising = 0
    Waiting_For_PEV = 1
    Negotiating_Connection = 2
    Connected_With_Full_Info = 3
    Insulation_Test = 4
    Precharge = 5
    Waiting_For_Charge = 6
    Charging = 7
    Ending_Charge = 8
    Welding_Detection = 9
    Closing_Communication = 10
    CCS_Authorisation_Process = 11
    Not_Available = 12

    def __str__(self) -> str:
        return self.name


class PowerFunction(enum.IntEnum):
    """Enumeration of power functions requested by the controller.
    Used in message [0x0x6B003] DC_Power_Control
    """

    Off = 0b0000
    Standby = 0b0001
    Insulation_Test = 0b0010
    Precharge = 0b0100
    Power_Transfer = 0b1000


class SetpointsMode(enum.IntEnum):
    """Enumeration of setpoints modes.
    Used in message [0x0x6B003] DC_Power_Control
    """

    Target_Mode = 0
    Range_Mode = 1


class SequenceFlags(enum.IntEnum):
    """Enumeration of flags controlling the charge sequence.
    Used in message [0x63002] Sequence_Control.
    Here we have them in little endian (LSB on the right).
    """

    NONE = 0
    Start_Charge_Authorisation = 0x000001
    CHAdeMO_Start_Button = 0x000002
    CCS_Authorisation_Done = 0x000100
    CCS_Authorisation_Valid = 0x000200
    Charge_Parameters_Done = 0x000400
    User_Stop_Button = 0x010000


class Simulator(can.Listener):
    """Simulator of power modules compatible with Advantics EVSE Generic CAN interface v3"""

    # Internal states definitions
    # From the controller
    communication_state: ControllerState
    target_voltage: float
    current_range_min: float
    current_range_max: float
    power_function: PowerFunction
    setpoints_mode: SetpointsMode
    close_output_contactors: bool
    lower_output_voltage: bool
    # For Power_Modules_Status message
    present_voltage: float
    present_current: float
    power_modules_temperature: int
    enclosure_temperature: int
    system_enable: bool
    insulation_resistance: int
    # For Sequence_Control message
    _sequence_flags: int

    def __init__(
            self,
            app: Application,
            *,
            pistol_index: int,
            ccs_authorisation_duration: float = 3,
            ccs_authorisation_success: bool = True,
            charge_parameters_negotiation_duration: float = 1.5,
            power_modules_wake_up_duration: float = 1,
            power_modules_dead_time: float = 1,
            voltage_ramp_up_slope: float = 200,
            voltage_ramp_down_slope: float = 100,
            current_ramp_up_slope: float = 20,
            current_ramp_down_slope: float = 20,
            charge_duration: float = 10,
            maximum_voltage: float = 500,
            maximum_charge_current: float = 120,
            maximum_discharge_current: float = 120,
    ) -> None:
        self._app = app
        self._bus = app.bus
        self._pistol_index = pistol_index

        # Simulation parameters
        self._ccs_authorisation_duration = ccs_authorisation_duration
        self._ccs_authorisation_success = ccs_authorisation_success
        self._charge_parameters_negotiation_duration = charge_parameters_negotiation_duration
        self._power_modules_wake_up_duration = power_modules_wake_up_duration
        self._power_modules_dead_time = power_modules_dead_time
        self._charge_duration = charge_duration
        self._ramps_simulator = RampSimulator(
            self,
            voltage_ramp_up_slope,
            voltage_ramp_down_slope,
            current_ramp_up_slope,
            current_ramp_down_slope,
            update_dt=0.1,
        )
        self._maximum_voltage: float = maximum_voltage
        self._maximum_charge_current: float = maximum_charge_current
        self._maximum_discharge_current: float = maximum_discharge_current
        self._dynamic_target_current: float = 0

        # Initialise internal states
        # Those assigned here are meant to be initialised once.
        # Whereas those in reset() are meant to be reinitialised after every charge.
        self._sequence_flags = 0
        self._slope_start_voltage = 0
        self._slope_start_current = 0
        self.communication_state = ControllerState.Initialising
        self._last_state = ControllerState.Initialising
        self.target_voltage = 0
        self.current_range_min = 0
        self.current_range_max = 0
        self.power_function = PowerFunction.Off
        self._last_power_function = PowerFunction.Off
        self.setpoints_mode = SetpointsMode.Target_Mode
        self.close_output_contactors = False
        self._last_close_output_contactors = False
        self.lower_output_voltage = False
        self.reset()

        # Periodic messages
        # Power_Modules_Status
        self._power_modules_status_msg = can.Message(
            arbitration_id=FrameID.Power_Modules_Status.for_pistol(self._pistol_index),
            is_extended_id=True,
            data=self.encode_power_modules_status(),
        )
        self._power_modules_status_task = self._bus.send_periodic(
            self._power_modules_status_msg,
            period=0.1,
            modifier_callback=self._send_callback_power_modules_status,
        )

        # DC_Power_Parameters
        self._dc_power_parameters_msg = can.Message(
            arbitration_id=FrameID.DC_Power_Parameters.for_pistol(self._pistol_index),
            is_extended_id=True,
            data=self.encode_dc_power_parameters(),
        )
        self._power_modules_status_task = self._bus.send_periodic(
            self._dc_power_parameters_msg,
            period=0.1,
            modifier_callback=self._send_callback_dc_power_parameters,
        )

    def start(self) -> None:
        self._ramps_simulator.start()

    def stop(self) -> None:
        self._ramps_simulator.stop()

    # Internal states

    def reset(self) -> None:
        """Reset our internal states"""

        # Power_Modules_Status
        self.present_voltage = 0
        self.present_current = 0
        self.power_modules_temperature = 20
        self.enclosure_temperature = 20
        self.system_enable = False
        self.insulation_resistance = 0

        # Sequence_Control
        flags = SequenceFlags.Start_Charge_Authorisation
        if self._ccs_authorisation_duration == 0:
            flags |= SequenceFlags.CCS_Authorisation_Done
            if self._ccs_authorisation_success:
                flags |= SequenceFlags.CCS_Authorisation_Valid
        if self._charge_parameters_negotiation_duration == 0:
            flags |= SequenceFlags.Charge_Parameters_Done
        self.sequence_flags = flags

        # Other internal
        self._insulation_test_start_at = 0
        self._precharge_start_at = 0
        self._charge_start_at = 0

    @property
    def sequence_flags(self) -> int:
        """Flags used in Sequence_Control message.
        If modified, a new Sequence_Control message is sent.
        """

        return self._sequence_flags

    @sequence_flags.setter
    def sequence_flags(self, value: int) -> None:
        if self._sequence_flags != value:
            self._sequence_flags = value
            self.send_sequence_control()

    # Messages reading and state processing

    def on_message_received(self, msg: can.Message) -> None:
        """Main callback for every CAN message received.

        Basic principle is we look for messages that interest us. When there is one,
        we decode it (NB: the decode functions directly change our internal state variables).

        For certain signals (controller state and power function) we look if they have changed.
        In which case we react to their changes.
        """

        # Messages from the controller
        if msg.arbitration_id == FrameID.Advantics_Controller_Status.for_pistol(self._pistol_index):
            self.decode_advantics_controller_status(msg.data)
            # Controller state changes are "for information only". But we can use them
            # for the non-powered part (ie. during session negotiation, to sequence a few things).
            if self._last_state != self.communication_state:
                self.update_state(self.communication_state)
                self._last_state = self.communication_state

        elif msg.arbitration_id == FrameID.DC_Power_Control.for_pistol(self._pistol_index):
            self.decode_dc_power_control(msg.data)
            # Power function is the main state we need to track for power modules.
            # But there are also setpoints and commands in it. So, react on every message.
            self.update_dc_power_control()

        # Messages to the controller
        # We listen for those only to update our own internal states in case
        # these are sent externally (eg. you playing around with sending some CAN messages).
        # You do not have to do the same in your implementation.

        elif msg.arbitration_id == FrameID.Power_Modules_Status.for_pistol(self._pistol_index):
            self.decode_power_modules_status(msg.data)

        elif msg.arbitration_id == FrameID.Sequence_Control.for_pistol(self._pistol_index):
            self.decode_sequence_control(msg.data)

        elif msg.arbitration_id == FrameID.OCPP_Control.for_pistol(self._pistol_index):
            self.decode_ocpp_control(msg.data)

    def update_state(self, new_state: ControllerState) -> None:
        """Uses [0x6B000] Advantics_Controller_Status.State to sequence non-powered things"""

        match new_state:
            case ControllerState.Not_Available:
                # The controller starts by default in Not_Available.
                # You need to send it once the Sequence_Control message with
                # flag Start_Charge_Authorisation set to 1.
                if self.sequence_flags & SequenceFlags.Start_Charge_Authorisation:
                    self.send_sequence_control()

            case ControllerState.Waiting_For_PEV:
                # This indicates we terminated a charge session (or controller just started).
                # Use it to reset our internal states.
                self.reset()
                print('--------------------------------------')

            case ControllerState.CCS_Authorisation_Process:
                # In CCS_Authorisation_Process we want to update sequence flags
                # with both CCS_Authorisation_Done, and CCS_Authorisation_Valid.
                if not (self.sequence_flags & SequenceFlags.CCS_Authorisation_Done):
                    print('Authorising user...')
                    call_later(self.simulate_ccs_authorisation, self._ccs_authorisation_duration)

            case ControllerState.Connected_With_Full_Info:
                # Here we have to update sequence flags with Charge_Parameters_Done
                if not (self.sequence_flags & SequenceFlags.Charge_Parameters_Done):
                    print('Determining charge parameters...')
                    call_later(
                        self.simulate_charge_parameters_done,
                        self._charge_parameters_negotiation_duration,
                    )

            case ControllerState.Insulation_Test:
                # Before we can receive setpoints for insulation test, we must signal
                # System_Enable to be 1. You could have it always set to 1 if you want.
                # But here we will use it for the intended feature of waiting
                # power modules wake-up.
                if not self.system_enable:
                    print('Waking-up power modules...')
                    call_later(
                        self.simulate_power_modules_wake_up,
                        self._power_modules_wake_up_duration,
                    )

            case ControllerState.Charging:
                # Fall back to charging at max current in the beginning of charging every time.
                # To be overridden by OCPP if wanted.
                self._dynamic_target_current = self.current_range_max
                print(f'Update dynamic target current as {self._dynamic_target_current}')
                # To end the simulation after a set time
                call_later(self.simulate_normal_charge_stop, self._charge_duration)

            case _:
                pass

    def simulate_ccs_authorisation(self) -> None:
        """Delayed callback to proceed with user authorisation process"""

        if self._ccs_authorisation_success:
            self.sequence_flags |= (
                    SequenceFlags.CCS_Authorisation_Done | SequenceFlags.CCS_Authorisation_Valid
            )
            print('User is authorised to charge')
        else:
            flags = self.sequence_flags
            flags |= SequenceFlags.CCS_Authorisation_Done
            flags &= ~SequenceFlags.CCS_Authorisation_Valid
            self.sequence_flags = flags
            print('User is not authorised to charge')

    def simulate_charge_parameters_done(self) -> None:
        """Delayed callback to declare our charge parameters (ie. max voltage and currents
        in [0x63001] DC_Power_Parameters) are stable and we can send them to the vehicle.
        """

        self.sequence_flags |= SequenceFlags.Charge_Parameters_Done
        print('Charge parameters done')

    def simulate_power_modules_wake_up(self) -> None:
        """Delayed callback to declare ourselves ready for power"""

        self.system_enable = True
        print('Power modules awake')

    def simulate_normal_charge_stop(self) -> None:
        """Delayed callback to stop the charge in a normal way, from charger side."""

        self.sequence_flags |= SequenceFlags.User_Stop_Button
        print('Stopping charge with normal stop request')

    def update_dc_power_control(self) -> None:
        """
        [0x6B003] DC_Power_Control is sent during any powered phase of a DC charge
        session. It contains the power function in use, setpoint targets
        or range, the setpoints mode, and the output contactors and voltage lowering commands.
        """

        match self.power_function:
            case PowerFunction.Off:
                self.handle_off()

            case PowerFunction.Standby:
                self.handle_standby()

            case PowerFunction.Insulation_Test:
                self.handle_insulation_test()

            case PowerFunction.Precharge:
                self.handle_precharge()

            case PowerFunction.Power_Transfer:
                self.handle_power_transfer()

        self._last_power_function = self.power_function

        # For contactors, just print out for information (can use the onboard relay actually)
        if self._last_close_output_contactors != self.close_output_contactors:
            self._last_close_output_contactors = self.close_output_contactors
            if self.close_output_contactors:
                print('Closing output contactors')
            else:
                print('Opening output contactors')

    def handle_off(self) -> None:
        """
        In off, power modules should not run any power processing functions.
        You will not be requested to activate a power processing function
        without going first through Standby.

        The output of the charger (exposed on pistol pins) has ~0 V, floating.
        """

        self.present_voltage = 0
        self.present_current = 0

    def handle_standby(self) -> None:
        """
        In standby, power modules should not run any power processing functions.
        But they should remain ready to receive future requests.

        Power modules output should be floating. A load (eg. battery)
        might still be connected to their output. Therefore, there might
        or might not be several hundred volts applied to it, depending on
        the current charging phase.

        In standby you might also be requested to lower your output voltage
        with [0x6B003] DC_Power_Control.Lower_Output_Voltage.
        In that case you are meant to actively discharge power modules output.
        NB: The controller will NOT request an output discharge when
        a battery might be connected to it.
        """

        def _0v_reached() -> bool:
            print('Voltage lowered to 0 V')
            return True

        self.present_current = 0
        if (
                self.lower_output_voltage
                and (self.present_voltage > 0)
                and self._ramps_simulator.target_voltage != 0
        ):
            print('Lowering output voltage...')
            # Set the target_current to zero to avoid the ramp sim from ramping back up to target from 0A
            self._ramps_simulator.set_target_current(0)
            self._ramps_simulator.set_target_voltage(0, reached_cb=_0v_reached)

    def handle_insulation_test(self) -> None:
        """
        In insulation test, power modules (or an external safety module)
        apply high voltage and measure the current leaked through ground.

        The test voltage reaches up to the vehicle through the pistol
        and vehicle inlet since our output contactors are closed at this point.
        However, vehicle input contactors are still open.

        At the end of the insulation test you will be asked to go to
        standby and lower power modules output voltage before going to precharge.
        """

        def _insulation_voltage_reached() -> bool:
            self.insulation_resistance = 100
            print(f'Voltage raised to {self.target_voltage:.1f} V')
            return True

        now = time.monotonic()
        if self._last_power_function != PowerFunction.Insulation_Test:
            self._insulation_test_start_at = now
            print(f'Ramping voltage to {self.target_voltage:.1f} V...')

        self._ramps_simulator.set_target_voltage(
            self.target_voltage,
            reached_cb=_insulation_voltage_reached,
            delay=(self._insulation_test_start_at + self._power_modules_dead_time) - now,
        )

    def handle_precharge(self) -> None:
        """
        In precharge, power modules match the requested target voltage,
        which should correspond to the vehicle present battery voltage.
        This is to avoid arcing of vehicle input contactors when they close.

        Precharge is specific to CCS. In CHAdeMO, you either have an output
        diode, or a custom precharge circuit around main output contactors.

        After precharge, once the vehicle deems the voltage to be matching,
        it will close its input contactors. During that time, you will be
        asked to go to standby without discharge, and leave power modules
        output floating to the vehicle battery voltage.
        """

        def _precharge_voltage_reached() -> bool:
            print(f'Voltage raised to {self.target_voltage:.1f} V')
            return True

        now = time.monotonic()
        if self._last_power_function != PowerFunction.Precharge:
            self._precharge_start_at = now
            print(f'Ramping voltage to {self.target_voltage:.1f} V...')

        self._ramps_simulator.set_target_voltage(
            self.target_voltage,
            reached_cb=_precharge_voltage_reached,
            delay=(self._precharge_start_at + self._power_modules_dead_time) - now,
        )

    def handle_power_transfer(self) -> None:
        """
        Main mode of operation, as power is being transferred in one way or another.

        For the sake of this tutorial simulator, we will only assume the simplest
        and most widely used behaviour. Which is operating in Target_Mode, and
        following Target_Voltage and Current_Range_Max as your setpoints.

        WARNING: The vehicle might not necessarily ramp up or down its requests.
        """

        # if in range_mode, get the target setpoint from _dynamic_target_current
        if self.setpoints_mode == SetpointsMode.Range_Mode:
            target_current = self._cap(self._dynamic_target_current, self.current_range_min, self.current_range_max)
        else:
            # Simple unidirectional power delivery on maximum current
            target_current = self.current_range_max

        def _charging_current_reached() -> bool:
            print(f'Current setpoint {target_current:.1f} A reached')
            return True

        now = time.monotonic()
        if self._last_power_function != PowerFunction.Power_Transfer:
            self._charge_start_at = now
            print(f'Ramping current to {target_current:.1f} A...')

        elif target_current != self._ramps_simulator.target_current:
            print(f'Changing current setpoint to {target_current:.1f} A')

        else:
            return

        self._ramps_simulator.set_target_current(
            target_current,
            reached_cb=_charging_current_reached,
            delay=(self._charge_start_at + self._power_modules_dead_time) - now,
        )

    # Message encoding, decoding, and sending methods

    def _cap(self, value: float, min_value: float, max_value: float) -> float:
        """Cap a given value to a minimum and maximum"""
        return min(max(value, min_value), max_value)

    # Advantics_Controller_Status

    def decode_advantics_controller_status(self, data: bytes | bytearray) -> None:
        (state,) = struct.unpack('<B', data)
        self.communication_state = ControllerState(state)

    # New_Charge_Session

    # Charge_Status_Change

    # DC_Power_Control

    def decode_dc_power_control(self, data: bytes | bytearray) -> None:
        target_voltage, current_max, current_min, modes = struct.unpack('<HhhB', data)

        self.target_voltage = target_voltage / 10
        self.current_range_max = current_max / 10
        # if self.current_range_max != 0:
        #     print(f'Current range max: {self.current_range_max=}, {current_max=}, {data=}')
        self.current_range_min = current_min / 10
        self.power_function = PowerFunction(modes & 0b0000_1111)
        self.setpoints_mode = SetpointsMode((modes & 0b0010_0000) >> 5)
        self.close_output_contactors = bool((modes & 0b0100_0000) >> 6)
        self.lower_output_voltage = bool((modes & 0b1000_0000) >> 7)

    # Charge_Session_Finished

    # Emergency_Stop

    # EV_Information_Battery

    # EV_Information_Voltages

    # EV_Information_Charge_Limits

    # EV_Information_Discharge_Limits

    # ADM_CS_SECC_Inputs

    # OCPP_Control
    def decode_ocpp_control(self, data: bytes | bytearray) -> None:
        (dynamic_target_current,) = struct.unpack('<h', data)
        self._dynamic_target_current = dynamic_target_current / 10
        print(f'Update dynamic target current as {self._dynamic_target_current}')

    # Power_Modules_Status

    def encode_power_modules_status(self) -> bytes:
        # print(f'> {self.system_enable}')
        return struct.pack(
            '<HhBBBB',
            self._cap(round(self.present_voltage * 10), 0, 65535),
            self._cap(round(self.present_current * 10), -32768, 32767),
            self._cap(round(self.power_modules_temperature + 40), 0, 255),
            self._cap(round(self.enclosure_temperature + 40), 0, 255),
            1 if self.system_enable else 0,
            self._cap(round(self.insulation_resistance / 2), 0, 255),
        )

    def decode_power_modules_status(self, data: bytes | bytearray) -> None:
        (
            present_voltage,
            present_current,
            power_modules_temperature,
            enclosure_temperature,
            system_enable,
            insulation_resistance,
        ) = struct.unpack('<HhBBBB', data)

        self.present_voltage = present_voltage / 10
        self.present_current = present_current / 10
        self.power_modules_temperature = power_modules_temperature - 40
        self.enclosure_temperature = enclosure_temperature - 40
        self.system_enable = bool(system_enable)
        self.insulation_resistance = insulation_resistance * 2

    def _send_callback_power_modules_status(self, msg: can.Message) -> None:
        """Callback provided to self._bus.send_periodic() for automatic update
        of message content"""
        msg.data = self.encode_power_modules_status()

    # DC_Power_Parameters

    def encode_dc_power_parameters(self) -> bytes:
        return struct.pack(
            '<HHHh',
            self._cap(round(self._maximum_voltage * 10), 0, 65535),
            self._cap(round(self._maximum_charge_current * 10), 0, 65535),
            self._cap(round(self._maximum_discharge_current * 10), 0, 65535),
            0,
        )

    def _send_callback_dc_power_parameters(self, msg: can.Message) -> None:
        """Callback provided to self._bus.send_periodic() for automatic update
        of message content"""
        msg.data = self.encode_dc_power_parameters()

    # Sequence_Control

    def encode_sequence_control(self) -> bytes:
        return self.sequence_flags.to_bytes(length=3, byteorder='little')

    def decode_sequence_control(self, data: bytes | bytearray) -> None:
        self._sequence_flags = int.from_bytes(data, byteorder='little')

    def send_sequence_control(self) -> None:
        msg = can.Message(
            arbitration_id=FrameID.Sequence_Control.for_pistol(self._pistol_index),
            is_extended_id=True,
            data=self.encode_sequence_control(),
        )
        self._bus.send(msg)

    # ADM_CS_SECC_Outputs


# Wrappers around threads to implement simulated behaviours


def call_later(callback: Callable[[], None], dt: float) -> Thread | None:
    """Simple delayed execution of code"""

    if dt <= 0:
        callback()
        return None

    def wrapper() -> None:
        try:
            time.sleep(dt)
            callback()
        except Exception as ex:  # noqa: BLE001
            print(f'Got exception in call_later thread for {callback}.', str(ex))

    thread = Thread(target=wrapper)
    thread.start()

    return thread


class RampSimulator:
    def __init__(
            self,
            simulator: Simulator,
            voltage_ramp_up: float,
            voltage_ramp_down: float,
            current_ramp_up: float,
            current_ramp_down: float,
            update_dt: float = 0.1,
    ) -> None:
        self._simulator = simulator
        self._update_dt = update_dt
        self._voltage_ramps = (voltage_ramp_up, voltage_ramp_down)
        self._current_ramps = (current_ramp_up, current_ramp_down)

        self._thread = Thread(target=self._run)
        self._stop_event = Event()

        self.target_voltage = 0.0
        self.target_current = 0.0
        self._delay = 0.0
        self.voltage_reached_cb: Callable[[], bool | None] | None = None
        self.current_reached_cb: Callable[[], bool | None] | None = None

    def set_target_voltage(
            self,
            target_voltage: float,
            reached_cb: Callable[[], bool | None] | None = None,
            delay: float = 0,
    ) -> None:
        self.voltage_reached_cb = reached_cb
        self._delay = time.monotonic() + delay
        self.target_voltage = target_voltage

    def set_target_current(
            self,
            target_current: float,
            reached_cb: Callable[[], bool | None] | None = None,
            delay: float = 0,
    ) -> None:
        self.current_reached_cb = reached_cb
        self._delay = time.monotonic() + delay
        self.target_current = target_current

    def start(self) -> None:
        self._stop_event.clear()
        self._thread.start()

    def _run(self) -> None:
        simulator = self._simulator
        dt = self._update_dt
        voltage_ramp_up, voltage_ramp_down = self._voltage_ramps
        current_ramp_up, current_ramp_down = self._current_ramps

        try:
            while not self._stop_event.wait(dt):
                if self._delay > time.monotonic():
                    continue

                # Voltage
                target_voltage = self.target_voltage
                present_voltage = simulator.present_voltage
                voltage_diff = target_voltage - present_voltage
                if voltage_diff >= 0:
                    present_voltage += min(voltage_ramp_up * dt, voltage_diff)
                else:
                    present_voltage += max(-voltage_ramp_down * dt, voltage_diff)
                simulator.present_voltage = present_voltage
                if voltage_diff and (present_voltage == target_voltage) and self.voltage_reached_cb:
                    delete = self.voltage_reached_cb()
                    if delete:
                        self.voltage_reached_cb = None

                # Current
                target_current = self.target_current
                present_current = simulator.present_current
                current_diff = target_current - present_current
                if current_diff >= 0:
                    present_current += min(current_ramp_up * dt, current_diff)
                else:
                    present_current += max(-current_ramp_down * dt, current_diff)
                simulator.present_current = present_current
                if current_diff and (present_current == target_current) and self.current_reached_cb:
                    delete = self.current_reached_cb()
                    if delete:
                        self.current_reached_cb = None

        except Exception as ex:  # noqa: BLE001
            print('Got exception in RampSimulator thread.', str(ex))

    def stop(self, timeout: float | None = None) -> None:
        self._stop_event.set()
        if timeout and (timeout < 0):
            return
        self._thread.join(timeout)


class Application:
    """Main application class. Handles creation of various objects, and life cycle of it."""

    def __init__(self, bus_config: can.typechecking.BusConfig, **simulator_config: Any) -> None:
        self._bus_config = bus_config
        self._simulator_config = simulator_config

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
            simulator = self._simulator = Simulator(self, **self._simulator_config)

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
        self.simulator.start()  # Because notifier does not do it (but it does for stop...)

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
        *,
        pistol_index: int = 1,
        ccs_authorisation_duration: float = 3,
        ccs_authorisation_success: bool = True,
        charge_parameters_negotiation_duration: float = 1.5,
        power_modules_wake_up_duration: float = 1,
        power_modules_dead_time: float = 1,
        voltage_ramp_up_slope: float = 200,
        voltage_ramp_down_slope: float = 100,
        current_ramp_up_slope: float = 20,
        current_ramp_down_slope: float = 20,
        charge_duration: float = 10,
        maximum_voltage: float = 500,
        maximum_charge_current: float = 120,
        maximum_discharge_current: float = 120,
) -> None:
    """Simulator of power modules compatible with Advantics EVSE Generic CAN interface v3"""
    try:
        bus_config = can.util.load_config(path=can_config)
    except can.exceptions.CanInterfaceNotImplementedError as ex:
        print(f'[red]ERROR:[/] Incorrect CAN configuration. {ex}.')
        raise typer.Abort from ex

    with Application(
            bus_config,
            pistol_index=pistol_index,
            ccs_authorisation_duration=ccs_authorisation_duration,
            ccs_authorisation_success=ccs_authorisation_success,
            charge_parameters_negotiation_duration=charge_parameters_negotiation_duration,
            power_modules_wake_up_duration=power_modules_wake_up_duration,
            power_modules_dead_time=power_modules_dead_time,
            voltage_ramp_up_slope=voltage_ramp_up_slope,
            voltage_ramp_down_slope=voltage_ramp_down_slope,
            current_ramp_up_slope=current_ramp_up_slope,
            current_ramp_down_slope=current_ramp_down_slope,
            charge_duration=charge_duration,
            maximum_voltage=maximum_voltage,
            maximum_charge_current=maximum_charge_current,
            maximum_discharge_current=maximum_discharge_current,
    ) as app:
        app.run()


if __name__ == '__main__':
    typer.run(cli_main)
