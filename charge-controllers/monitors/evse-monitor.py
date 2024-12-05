# Copyright (c) 2024 ADVANTICS SAS
# Original author: Axel Voitier
# Part of Advantics examples
# MIT licensed
#
# spell-checker:words setpoint setpoints estop renderable colors
# spell-checker:ignore cantools EVSE SECC SLAC OCPP CHAdeMO exctype excinst exctb asciichartpy
from __future__ import annotations

# System imports
import os
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING
from datetime import datetime

# Third-party imports
import asciichartpy
import can
import cantools.database
import typer
from rich import box, print
from rich.console import Console
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

# Local imports

if TYPE_CHECKING:
    from types import TracebackType
    from typing import Any, Self

    from rich.console import ConsoleOptions, RenderResult


@dataclass
class ChargerControl:
    power_function: str
    setpoints_mode: str
    output_contactors: str
    lower_output_voltage: str
    target_voltage: float
    current_range_max: float
    current_range_min: float
    ocpp_target_current: float


@dataclass
class ChargerStatus:
    present_voltage: float
    present_current: float
    power_modules_temp: int
    enclosure_temp: int
    system_enable: str
    insulation_resistance: int
    digital_inputs: str
    cpu_temp: int
    pistol_ptc1: int
    pistol_ptc2: int


@dataclass
class ChargerParameters:
    maximum_voltage: float
    maximum_charge_current: float
    maximum_discharge_current: float
    range_target_current: float
    ###
    start_charge_authorisation: str
    chademo_start_button: str
    ccs_authorisation_done: str
    ccs_authorisation_valid: str
    charge_parameters_done: str
    user_stop_button: str
    ###
    digital_outputs: str


@dataclass
class SessionStatus:
    state: str
    protocol: str
    pins: str
    stop_state: str
    estop_origin: str


@dataclass
class VehicleStatus:
    ready: str
    battery_capacity: int
    soc: int
    min_voltage: float
    max_voltage: float
    min_charge_current: float
    max_charge_current: float
    min_charge_power: int
    max_charge_power: int
    min_discharge_current: float
    max_discharge_current: float
    min_discharge_power: int
    max_discharge_power: int


enable_can_log = False
logged_messages = {}
log_filename = f"./evse_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log"


def log_can_msg(msg_name, signals, senders=None):
    if not enable_can_log:
        return

    if msg_name in logged_messages:
        if logged_messages[msg_name] == signals:
            # The same message is logged with the same body last time, do not repeat
            return

    with open(log_filename, 'a') as file:
        file.write(f"{datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")}\t{senders}\t{msg_name}\t{signals}\n")
    logged_messages[msg_name] = signals


class AdvanticsEVSEInterfaceV3(can.Listener):
    def __init__(self, app: Application, pistol_index: int) -> None:
        self._app = app
        self._bus = app.bus
        self._index = pistol_index
        self._db: cantools.database.Database = cantools.database.load_file(
            'Advantics_Generic_EVSE_protocol_v3.kcd',
        )  # type: ignore[reportAttributeAccessIssue]

        self._bus.set_filters(
            [
                can.typechecking.CanFilterExtended(
                    can_id=message.frame_id | ((self._index & 0x0F) << 24),
                    can_mask=0x1FFFFFFF,
                    extended=True,
                )
                for message in self._db.messages
                # if message.frame_id & (1 << 15)
            ],
        )

        self.charger_control = ChargerControl(
            power_function='----',
            setpoints_mode='----',
            output_contactors='----',
            lower_output_voltage='----',
            target_voltage=0,
            current_range_max=0,
            current_range_min=0,
            ocpp_target_current=0,
        )
        self._app.update_charger_control(self.charger_control)

        self.charger_status = ChargerStatus(
            present_voltage=0,
            present_current=0,
            power_modules_temp=-40,
            enclosure_temp=-40,
            system_enable='----',
            insulation_resistance=0,
            digital_inputs='----',
            cpu_temp=-40,
            pistol_ptc1=-40,
            pistol_ptc2=-40,
        )
        self._app.update_charger_status(self.charger_status)

        self.charger_parameters = ChargerParameters(
            maximum_voltage=0,
            maximum_charge_current=0,
            maximum_discharge_current=0,
            range_target_current=0,
            start_charge_authorisation='----',
            chademo_start_button='----',
            ccs_authorisation_done='----',
            ccs_authorisation_valid='----',
            charge_parameters_done='----',
            user_stop_button='----',
            digital_outputs='----',
        )
        self._app.update_charger_parameters(self.charger_parameters)

        self.session_status = SessionStatus(
            state='----',
            protocol='----',
            pins='----',
            stop_state='----',
            estop_origin='----',
        )
        self._app.update_session_status(self.session_status)

        self.vehicle_status = VehicleStatus(
            ready='----',
            battery_capacity=0,
            soc=0,
            min_voltage=0,
            max_voltage=0,
            min_charge_current=0,
            max_charge_current=0,
            min_charge_power=0,
            max_charge_power=0,
            min_discharge_current=0,
            max_discharge_current=0,
            min_discharge_power=0,
            max_discharge_power=0,
        )
        self._app.update_vehicle_status(self.vehicle_status)

    def on_message_received(self, msg: can.Message) -> None:  # noqa: C901, PLR0912, PLR0915
        try:
            message = self._db.get_message_by_frame_id(msg.arbitration_id & 0xFFFFFF)
        except KeyError:
            return
        signals = message.decode(msg.data)

        log_can_msg(message.name, signals, message.senders)

        # Messages sent by the controller

        if message.name == 'Advantics_Controller_Status':
            self.session_status.state = str(signals['State'])
            self._app.update_session_status(self.session_status)

        elif message.name == 'New_Charge_Session':
            self.session_status.protocol = str(signals['Communication_Protocol'])
            self.session_status.pins = str(signals['Plug_and_pins'])
            self._app.update_session_status(self.session_status)

        elif message.name == 'Charge_Status_Change':
            self.vehicle_status.ready = str(signals['Vehicle_Ready_for_Charging'])
            self._app.update_vehicle_status(self.vehicle_status)

        elif message.name == 'DC_Power_Control':
            self.charger_control.power_function = str(signals['Power_Function'])
            self.charger_control.setpoints_mode = str(signals['Setpoints_Mode'])
            self.charger_control.output_contactors = str(signals['Output_Contactors'])
            self.charger_control.lower_output_voltage = str(signals['Lower_Output_Voltage'])
            self.charger_control.target_voltage = float(signals['Target_Voltage'])
            self.charger_control.current_range_max = float(signals['Current_Range_Max'])
            self.charger_control.current_range_min = float(signals['Current_Range_Min'])
            self._app.update_charger_control(self.charger_control)

        elif message.name == 'Charge_Session_Finished':
            self.session_status.stop_state = str(signals['State'])
            self._app.update_session_status(self.session_status)

        elif message.name == 'Emergency_Stop':
            self.session_status.estop_origin = str(signals['Origin'])
            self._app.update_session_status(self.session_status)

        elif message.name == 'EV_Information_Battery':
            self.vehicle_status.battery_capacity = int(signals['Battery_Capacity'])
            self.vehicle_status.soc = int(signals['Present_State_of_Charge'])
            self._app.update_vehicle_status(self.vehicle_status)

        elif message.name == 'EV_Information_Voltages':
            self.vehicle_status.min_voltage = float(signals['EV_Minimum_Voltage'])
            self.vehicle_status.max_voltage = float(signals['EV_Maximum_Voltage'])
            self._app.update_vehicle_status(self.vehicle_status)

        elif message.name == 'EV_Information_Charge_Limits':
            self.vehicle_status.min_charge_current = float(signals['EV_Minimum_Charge_Current'])
            self.vehicle_status.max_charge_current = float(signals['EV_Maximum_Charge_Current'])
            self.vehicle_status.min_charge_power = int(signals['EV_Minimum_Charge_Power'])
            self.vehicle_status.max_charge_power = int(signals['EV_Maximum_Charge_Power'])
            self._app.update_vehicle_status(self.vehicle_status)

        elif message.name == 'EV_Information_Discharge_Limits':
            self.vehicle_status.min_discharge_current = float(
                signals['EV_Minimum_Discharge_Current']
            )
            self.vehicle_status.max_discharge_current = float(
                signals['EV_Maximum_Discharge_Current']
            )
            self.vehicle_status.min_discharge_power = int(signals['EV_Minimum_Discharge_Power'])
            self.vehicle_status.max_discharge_power = int(signals['EV_Maximum_Discharge_Power'])
            self._app.update_vehicle_status(self.vehicle_status)

        elif message.name == 'ADM_CO_CUI1_Inputs':
            self.charger_status.digital_inputs = (
                f'0:{"H" if signals['SWITCH0'] else "L"} '
                f'1:{"H" if signals['SWITCH1'] else "L"} '
                f'2:{"H" if signals['SWITCH2'] else "L"} '
                f'3:{"H" if signals['SWITCH3'] else "L"} '
                f'4:{"H" if signals['SWITCH4'] else "L"} '
                f'5:{"H" if signals['SWITCH5'] else "L"}'
            )
            self.charger_status.cpu_temp = int(signals['CPU_Temperature'])
            self.charger_status.pistol_ptc1 = int(signals['Pistol_PTC1'])
            self.charger_status.pistol_ptc2 = int(signals['Pistol_PTC2'])
            self._app.update_charger_status(self.charger_status)

        elif message.name == 'ADM_CS_SECC_Inputs':
            self.charger_status.digital_inputs = (
                f'1:{"H" if signals['Digital_Input1'] else "L"} '
                f'2:{"H" if signals['Digital_Input2'] else "L"} '
                f'3:{"H" if signals['Digital_Input3'] else "L"} '
                f'4:{"H" if signals['Digital_Input4'] else "L"}'
            )
            self.charger_status.cpu_temp = int(signals['CPU_Temperature'])
            self.charger_status.pistol_ptc1 = int(signals['Pistol_PTC1'])
            self.charger_status.pistol_ptc2 = int(signals['Pistol_PTC2'])
            self._app.update_charger_status(self.charger_status)

        elif message.name == 'OCPP_Control':
            self.charger_control.ocpp_target_current = float(signals['Dynamic_Target_Current'])
            self._app.update_charger_control(self.charger_control)

        # Do also the messages going to the controller
        # in case we are just watching someone else session

        elif message.name == 'Power_Modules_Status':
            self.charger_status.present_voltage = float(signals['Present_Voltage'])
            self.charger_status.present_current = float(signals['Present_Current'])
            self.charger_status.power_modules_temp = int(signals['Power_Modules_Temperature'])
            self.charger_status.enclosure_temp = int(signals['Enclosure_Temperature'])
            self.charger_status.system_enable = str(signals['System_Enable'])
            self.charger_status.insulation_resistance = int(signals['Insulation_Resistance'])
            self._app.update_charger_status(self.charger_status)

        elif message.name == 'DC_Power_Parameters':
            self.charger_parameters.maximum_voltage = float(signals['Maximum_Voltage'])
            self.charger_parameters.maximum_charge_current = float(
                signals['Maximum_Charge_Current']
            )
            self.charger_parameters.maximum_discharge_current = float(
                signals['Maximum_Discharge_Current']
            )
            self.charger_parameters.range_target_current = float(signals['Range_Target_Current'])
            self._app.update_charger_parameters(self.charger_parameters)

        elif message.name == 'Sequence_Control':
            self.charger_parameters.start_charge_authorisation = str(
                signals['Start_Charge_Authorisation']
            )
            self.charger_parameters.chademo_start_button = str(signals['CHAdeMO_Start_Button'])
            self.charger_parameters.ccs_authorisation_done = str(signals['CCS_Authorisation_Done'])
            self.charger_parameters.ccs_authorisation_valid = str(
                signals['CCS_Authorisation_Valid']
            )
            self.charger_parameters.charge_parameters_done = str(signals['Charge_Parameters_Done'])
            self.charger_parameters.user_stop_button = str(signals['User_Stop_Button'])
            self._app.update_charger_parameters(self.charger_parameters)

        elif message.name == 'ADM_CS_SECC_Outputs':
            self.charger_parameters.digital_outputs = (
                f'1:{"H" if signals['Digital_Output1'] else "L"} '
                f'2:{"H" if signals['Digital_Output2'] else "L"} '
                f'3:{"H" if signals['Digital_Output3'] else "L"} '
                f'4:{"H" if signals['Digital_Output4'] else "L"}'
            )
            self._app.update_charger_parameters(self.charger_parameters)

        self._app.update_wait_on(
            self.charger_control,
            self.charger_status,
            self.charger_parameters,
            self.session_status,
            self.vehicle_status,
        )
        # self._app.update_graph(self.charger_control, self.charger_status)

    def on_error(self, exc: Exception) -> None:
        self._app.live.stop()
        raise exc


class RenderableConsole(Console):
    def __init__(self) -> None:
        super().__init__(record=True, file=open(os.devnull, 'w'))  # noqa: PTH123, SIM115

    def __rich_console__(self, console: Console, options: ConsoleOptions) -> RenderResult:
        texts = self.export_text(clear=False).split('\n')
        yield from texts[-options.height:]


class PlotPanel:
    def __init__(
            self,
            app: Application,
            kind: str,
            unit: str,
            color: str = '',
            ratio: int = 2,
    ) -> None:
        self._app = app
        self._data: deque[float] = deque(maxlen=500)
        self._kind = kind
        self._unit = unit
        self._color = color
        self._asciichartpy_color = getattr(asciichartpy, self._color) if self._color else None
        self._ratio = ratio

        self._config: dict[str, str | int | float | list[str | None]] = dict(
            # min=0,
            colors=[
                self._asciichartpy_color,
            ],
        )

    def __rich_console__(self, console: Console, options: ConsoleOptions) -> RenderResult:
        evse = self._app.evse
        kind = self._kind
        config = self._config
        ratio = self._ratio

        sample: float = getattr(evse.charger_status, f'present_{kind}')
        self._data.append(sample)

        if kind == 'voltage':
            max_data = min(evse.vehicle_status.max_voltage, evse.charger_parameters.maximum_voltage)
            config['max'] = max_data if max_data else 500
            config['min'] = 0
        elif kind == 'current':
            max_data = min(
                evse.vehicle_status.max_charge_current,
                evse.charger_parameters.maximum_charge_current,
            )
            config['max'] = max_data if max_data else 200

            min_data = -min(
                evse.vehicle_status.max_discharge_current,
                evse.charger_parameters.maximum_discharge_current,
            )
            config['min'] = min_data if min_data else 0

        config['height'] = options.max_height - 1
        config['format'] = '{:4.0f} ' + self._unit
        width = (options.max_width - 7) * ratio

        plot = asciichartpy.plot(list(self._data)[-width::ratio], config)
        if self._color:
            plot = plot.replace(self._asciichartpy_color, f'[{self._color}]')
            plot = plot.replace(asciichartpy.reset, '[/]')
        yield plot


class Application:
    def __init__(self, bus_config: can.typechecking.BusConfig, **interface_config: Any) -> None:
        self._bus_config = bus_config
        self._interface_config = interface_config
        self._bus: can.BusABC | None = None
        self._notifier: can.Notifier | None = None
        self._evse: AdvanticsEVSEInterfaceV3 | None = None
        # self._emulator: Emulator | None = None

        self._previous_status = ''
        self.console = RenderableConsole()
        self.layout = self._make_layout()
        self.live = Live(self.layout, screen=True, redirect_stderr=True)

    def _make_layout(self) -> Layout:
        layout = Layout(name='EVSE')
        layout.split_column(
            Layout(name='Hints', size=1),
            Layout(name='First'),
            Layout(name='Second'),
            Layout(name='Graphs'),
            # Layout(self.console, name='Console', size=2),
        )
        layout['First'].split_row(
            Layout(name='charger-control'),
            Layout(name='charge-session'),
            Layout(name='vehicle-status'),
        )
        layout['Second'].split_row(
            Layout(name='charger-status'),
            Layout(name='charger-parameters'),
            Layout(Panel(self.console, title='Console'), name='placeholder1'),
            # Layout(Panel(PlotPanel(self)), name='graph'),
        )
        layout['Graphs'].split_row(
            Layout(
                Panel(PlotPanel(self, 'voltage', 'V', 'yellow'), title='Present Voltage'),
                name='graph-voltage',
            ),
            Layout(
                Panel(PlotPanel(self, 'current', 'A', 'blue'), title='Present Current'),
                name='graph-current',
            ),
        )
        layout['Hints'].update(Text('', style='black on white'))

        return layout

    @property
    def bus(self) -> can.BusABC:
        if (bus := self._bus) is None:
            bus = self._bus = can.Bus(**self._bus_config)

        return bus

    @property
    def evse(self) -> AdvanticsEVSEInterfaceV3:
        if (evse := self._evse) is None:
            evse = self._evse = AdvanticsEVSEInterfaceV3(self, **self._interface_config)

        return evse

    # @property
    # def emulator(self) -> Emulator:
    #     if (emulator := self._emulator) is None:
    #         emulator = self._emulator = Emulator(self, **self._interface_config)

    #     return emulator

    @property
    def notifier(self) -> can.Notifier:
        if (notifier := self._notifier) is None:
            # notifier = self._notifier = can.Notifier(self.bus, [self.evse, self.emulator])
            notifier = self._notifier = can.Notifier(self.bus, [self.evse])

        return notifier

    def start(self) -> None:
        _ = self.notifier  # Implicitly instantiate bus, evse, and notifier, which starts-up

    def shutdown(self) -> None:
        self.live.stop()

        if self._notifier is not None:
            self._notifier.stop()
            self._notifier = None

        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None

    def __enter__(self) -> Self:
        self.start()
        return self

    def __exit__(
            self,
            exctype: type[BaseException] | None,
            excinst: BaseException | None,
            exctb: TracebackType | None,
    ) -> bool:
        self.shutdown()
        return False

    def display(self) -> None:
        with self.live:
            self.live._refresh_thread.join()  # type: ignore[reportPrivateUsage]

    def update_charger_control(self, data: ChargerControl) -> None:
        table = Table.grid(expand=True)
        table.box = box.HORIZONTALS
        table.add_column()
        table.add_column()
        table.add_row('[b]Power function:[/]', data.power_function)
        table.add_row('[b]Setpoints mode:[/]', data.setpoints_mode)
        table.add_section()
        table.add_row('[b]Output contactors:[/]', data.output_contactors)
        table.add_row('[b]Lower output voltage:[/]', data.lower_output_voltage)
        table.add_section()
        table.add_row('[b]Target voltage:[/]', f'{data.target_voltage:.2f} V')
        table.add_row('[b]Current range max:[/]', f'{data.current_range_max:.2f} A')
        table.add_row('[b]Current range min:[/]', f'{data.current_range_min:.2f} A')
        table.add_section()
        table.add_row('[b]OCPP target current:[/]', f'{data.ocpp_target_current:.2f} A')
        self.layout['First']['charger-control'].update(Panel(table, title='Charger Control'))

    def update_charger_status(self, data: ChargerStatus) -> None:
        table = Table.grid(expand=True)
        table.box = box.HORIZONTALS
        table.add_column()
        table.add_column()
        table.add_row('[b]Present voltage:[/]', f'{data.present_voltage:.2f} V')
        table.add_row('[b]Present current:[/]', f'{data.present_current:.2f} A')
        table.add_row('[b]Insulation resistance:[/]', f'{data.insulation_resistance} kΩ')
        table.add_row('[b]System enable:[/]', self._color_flag(data.system_enable))
        table.add_section()
        table.add_row('[b]Pistol PTC 1:[/]', f'{data.pistol_ptc1} °C')
        table.add_row('[b]Pistol PTC 2:[/]', f'{data.pistol_ptc2} °C')
        table.add_row('[b]Power modules temperature:[/]', f'{data.power_modules_temp} °C')
        table.add_row('[b]Enclosure temperature:[/]', f'{data.enclosure_temp} °C')
        table.add_row('[b]CPU temperature:[/]', f'{data.cpu_temp} °C')
        table.add_row('[b]Digital inputs:[/]', self._color_logic(data.digital_inputs))
        self.layout['Second']['charger-status'].update(Panel(table, title='Charger Status'))

    def update_charger_parameters(self, data: ChargerParameters) -> None:
        table = Table.grid(expand=True)
        table.box = box.HORIZONTALS
        table.add_column()
        table.add_column()
        table.add_row('[b]Maximum voltage:[/]', f'{data.maximum_voltage:.2f} V')
        table.add_row('[b]Maximum charge current:[/]', f'{data.maximum_charge_current:.2f} A')
        table.add_row('[b]Maximum discharge current:[/]', f'{data.maximum_discharge_current:.2f} A')
        table.add_row('[b]Range target current:[/]', f'{data.range_target_current:.2f} A')
        table.add_section()
        table.add_row(
            '[b]Start charge authorisation:[/]',
            self._color_flag(data.start_charge_authorisation),
        )
        table.add_row('[b]CHAdeMO start button:[/]', self._color_flag(data.chademo_start_button))
        table.add_row(
            '[b]CCS authorisation done:[/]',
            self._color_flag(data.ccs_authorisation_done),
        )
        table.add_row(
            '[b]CCS authorisation valid:[/]',
            self._color_flag(data.ccs_authorisation_valid, not_prefix='In'),
        )
        table.add_row(
            '[b]Charge parameters done:[/]',
            self._color_flag(data.charge_parameters_done),
        )
        table.add_row('[b]User stop button:[/]', self._color_flag(data.user_stop_button))
        table.add_section()
        table.add_row('[b]Digital outputs:[/]', self._color_logic(data.digital_outputs))
        self.layout['Second']['charger-parameters'].update(Panel(table, title='Charger Parameters'))

    def update_session_status(self, data: SessionStatus) -> None:
        table = Table.grid(expand=True)
        table.box = box.HORIZONTALS
        table.add_column()
        table.add_column()
        table.add_row('[b]State:[/]', data.state)
        table.add_row('[b]Protocol:[/]', data.protocol)
        table.add_row('[b]Pins:[/]', data.pins)
        table.add_section()
        table.add_row('[b]Stop state:[/]', data.stop_state)
        table.add_row('[b]E-Stop origin:[/]', data.estop_origin)
        self.layout['First']['charge-session'].update(Panel(table, title='Charge Session'))

    def update_vehicle_status(self, data: VehicleStatus) -> None:
        table = Table.grid(expand=True)
        table.box = box.HORIZONTALS
        table.add_column()
        table.add_column()
        table.add_row('[b]Ready:[/]', data.ready)
        table.add_section()
        table.add_row('[b]Battery capacity:[/]', f'{data.battery_capacity} kWh')
        table.add_row('[b]State of charge:[/]', f'{data.soc} %')
        table.add_section()
        table.add_row('[b]Min voltage:[/]', f'{data.min_voltage:0.2f} V')
        table.add_row('[b]Max voltage:[/]', f'{data.max_voltage:0.2f} V')
        table.add_section()
        table.add_row('[b]Min charge current:[/]', f'{data.min_charge_current:0.2f} A')
        table.add_row('[b]Max charge current:[/]', f'{data.max_charge_current:0.2f} A')
        table.add_row('[b]Min charge power:[/]', f'{data.min_charge_power} kW')
        table.add_row('[b]Max charge power:[/]', f'{data.max_charge_power} kW')
        table.add_section()
        table.add_row('[b]Min discharge current:[/]', f'{data.min_discharge_current:0.2f} A')
        table.add_row('[b]Max discharge current:[/]', f'{data.max_discharge_current:0.2f} A')
        table.add_row('[b]Min discharge power:[/]', f'{data.min_discharge_power} kW')
        table.add_row('[b]Max discharge power:[/]', f'{data.max_discharge_power} kW')
        self.layout['First']['vehicle-status'].update(Panel(table, title='Vehicle Status'))

    def update_wait_on(  # noqa: C901, PLR0912, PLR0915
            self,
            charger_control: ChargerControl,
            charger_status: ChargerStatus,
            charger_parameters: ChargerParameters,
            session_status: SessionStatus,
            vehicle_status: VehicleStatus,
    ) -> None:
        def _update_status(text: str) -> None:
            self.layout['Hints'].update(Text(text, style='black on white'))
            if text != self._previous_status:
                self.console.print(text)
                self._previous_status = text

        def _controller_busy() -> None:
            _update_status('Controller is doing its thing, wait a moment')

        def _unknown_state(extra: str = '') -> None:
            if extra:
                _update_status(f'Unknown state: {extra}')
            else:
                _update_status('Unknown state')

        if session_status.state == '----':
            _update_status('Waiting for CAN data from controller')

        elif session_status.state == 'Initialising':
            _update_status('Controller is starting-up')

        elif session_status.state == 'Not_Available':
            if charger_parameters.start_charge_authorisation != 'Allowed':
                _update_status(
                    'Pistol in not available state. '
                    'Waiting for Sequence_Control.Start_Charge_Authorisation = 1.',
                )
            else:
                _controller_busy()

        elif session_status.state == 'Waiting_For_PEV':
            _update_status('Waiting for vehicle to plug in')

        elif session_status.state == 'Negotiating_Connection':
            _update_status(
                'Waiting for initial communication setup (SLAC pairing, SDP, AppProtocol, '
                'and other handshakes)',
            )

        elif session_status.state == 'CCS_Authorisation_Process':
            if charger_parameters.ccs_authorisation_done != 'Done':
                _update_status(
                    'Doing CCS authorisation. '
                    'Waiting for Sequence_Control.CCS_Authorisation_Done = 1.',
                )
            else:
                _controller_busy()

        elif session_status.state == 'Connected_With_Full_Info':
            if charger_parameters.charge_parameters_done != 'Done':
                _update_status('Waiting for Sequence_Control.Charge_Parameters_Done = 1')
            else:
                _controller_busy()

        elif session_status.state == 'Insulation_Test':
            if charger_control.power_function == 'Off':
                if charger_status.system_enable != 'Allowed':
                    _update_status(
                        'Power modules wake-up. '
                        'Waiting for Power_Modules_Status.System_Enable = 1.',
                    )
                else:
                    _controller_busy()

            elif charger_control.power_function == 'Insulation_Test':
                min_present_voltage = charger_control.target_voltage * 0.9
                if charger_status.present_voltage >= min_present_voltage:
                    # Threshold at 100 Ohms / V
                    min_insulation = charger_status.present_voltage * 100 / 1000
                    if charger_status.insulation_resistance < min_insulation:
                        _update_status(
                            f'Insulation resistance < {min_insulation:.1f} kΩ. '
                            'Waiting for insulation resistance to be higher.',
                        )
                    else:
                        _update_status('Waiting for stabilised insulation resistance')

                else:
                    _update_status(
                        f'Present voltage is < {min_present_voltage:.1f} V. '
                        'Waiting for present voltage to ramp-up.',
                    )

            elif charger_control.power_function == 'Standby':
                if charger_control.lower_output_voltage == 'Lowering':
                    if charger_status.present_voltage > 20:
                        _update_status('Waiting for present voltage to go below 20 V.')
                    else:
                        _controller_busy()
                else:
                    _controller_busy()

            elif charger_control.power_function == 'Precharge':
                pass  # Ignore fluke

            else:
                _unknown_state(f'{session_status.state=}, {charger_control.power_function=}')

        elif session_status.state == 'Precharge':
            if charger_control.power_function == 'Precharge':
                min_voltage = charger_control.target_voltage - 20
                max_voltage = charger_control.target_voltage + 20
                if min_voltage <= charger_status.present_voltage <= max_voltage:
                    _update_status('Precharge voltage is matched. Waiting for vehicle to continue.')
                else:
                    _update_status(
                        'Waiting for present voltage to match '
                        f'{charger_control.target_voltage:.1f} V by +/- 20 V',
                    )

            elif charger_control.power_function == 'Standby':
                _update_status(
                    'Vehicle should have closed its contactors. '
                    'Waiting for it to start charging.',
                )

            else:
                _unknown_state(f'{session_status.state=}, {charger_control.power_function=}')

        elif session_status.state == 'Waiting_For_Charge':
            _update_status('Waiting for vehicle to start charging.')

        elif session_status.state == 'Charging':
            if charger_status.present_voltage < 20:
                _update_status('Output voltage is abnormally low. Are contactors still closed?')

            elif charger_control.power_function == 'Power_Transfer':
                direction = 'Charging' if charger_status.present_current >= 0 else 'Discharging'

                if charger_control.setpoints_mode == 'Target_Mode':
                    target_current = charger_control.current_range_max
                    if (target_current * 0.8) <= charger_status.present_current <= (target_current * 1.1):
                        _update_status(f'{direction}! Waiting for charge to stop.')
                    else:
                        _update_status(
                            f'In {direction.lower()} phase, '
                            'but you do not seem to track the current setpoint '
                            f'({target_current:.1f} A)',
                        )

                elif charger_control.setpoints_mode == 'Range_Mode':
                    if charger_status.present_current > charger_control.current_range_max:
                        _update_status(
                            f'{direction}, but present current is above maximum!'
                            f'({charger_control.current_range_max:.1f} A)',
                        )
                    elif charger_status.present_current < charger_control.current_range_min:
                        _update_status(
                            f'{direction}, but present current is below minimum!'
                            f'({charger_control.current_range_min:.1f} A)',
                        )
                    else:
                        _update_status(f'{direction}! Waiting for charge to stop.')

                else:
                    _unknown_state(
                        f'{session_status.state=}, '
                        f'{charger_control.power_function=}, '
                        f'{charger_control.setpoints_mode=}',
                    )

            elif charger_control.power_function == 'Standby':
                _update_status(
                    'In charging phase. But waiting for vehicle to make a non-zero request',
                )

            else:
                _unknown_state(f'{session_status.state=}, {charger_control.power_function=}')

        elif session_status.state == 'Ending_Charge':
            if charger_control.output_contactors == 'Close':
                if charger_status.present_current >= 1:
                    _update_status('Waiting for output current to lower below 1 A')
                else:
                    _controller_busy()

            elif charger_control.output_contactors == 'Open':
                if charger_status.present_voltage > 20:
                    _update_status('You should start lowering power modules output voltage')
                else:
                    _controller_busy()

            else:
                _unknown_state(f'{session_status.state=}, {charger_control.output_contactors=}')

        elif session_status.state == 'Welding_Detection':
            if charger_status.present_voltage > 20:
                _update_status('You should lower power modules output voltage')
            else:
                _update_status('Vehicle is doing welding detection. Waiting for it to finish.')

        elif session_status.state == 'Closing_Communication':
            if charger_status.present_voltage > 20:
                _update_status('You should lower power modules output voltage')
            else:
                _update_status('Waiting for vehicle to unplug')

        else:
            _unknown_state(f'{session_status.state=}')

    def _color_flag(self, flag: str, not_prefix: str = 'Not_', *, invert: bool = False) -> str:
        if flag == '----':
            return flag
        if flag.startswith(not_prefix) != invert:
            return f'[red]{flag}[/]'
        else:
            return f'[green]{flag}[/]'

    def _color_logic(self, ios: str) -> str:
        return ios.replace('H', '[green]H[/]').replace('L', '[red]L[/]')


def cli_main(
        can_config: Path = Path('can.conf'),
        pistol_index: int = 1,
        enable_can_logging: bool = False
) -> None:
    global enable_can_log
    enable_can_log = enable_can_logging
    try:
        bus_config = can.util.load_config(path=can_config)
    except can.exceptions.CanInterfaceNotImplementedError as ex:
        print(f'[red]ERROR:[/] Incorrect CAN configuration. {ex}.')
        raise typer.Abort from ex

    with Application(bus_config, pistol_index=pistol_index) as app:
        app.display()


if __name__ == '__main__':
    typer.run(cli_main)
