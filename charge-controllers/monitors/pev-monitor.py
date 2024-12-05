# Copyright (c) 2024 ADVANTICS SAS
# Original author: Axel Voitier
# Part of Advantics examples
# MIT licensed
#
# spell-checker:words renderable
# spell-checker:ignore cantools EVSE EVCC exctype excinst exctb
from __future__ import annotations

# System imports
import os
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING
from datetime import datetime

# Third-party imports
import can
import cantools.database
import typer
from rich import box, print
from rich.console import Console
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table

# Local imports

if TYPE_CHECKING:
    from types import TracebackType
    from typing import Any, Self

    from rich.console import ConsoleOptions, RenderResult


@dataclass
class VehicleControl:
    ac_evse_is_ready: str
    dc_close_contactors: str
    ###
    dc_contactor_pos_fb: str
    dc_contactor_neg_fb: str
    digital_inputs: str
    stop_charge: str
    ptc0: int
    ptc1: int
    ptc2: int
    cpu_temp: int
    can_sensor_temp: int


@dataclass
class ChargerStatus:
    max_current: float
    rcd_status: str


@dataclass
class VehicleParameters:
    target_energy_request: float
    min_energy_request: float
    max_energy_request: float
    min_v2x_energy_request: float
    max_v2x_energy_request: float
    departure_time: int


@dataclass
class SessionStatus:
    communication_stage: str
    protocol: str
    pins: str
    cp_duty_cycle: int
    cp_top_voltage: float
    cp_state: str
    pp_resistance: int
    inlet_lock_state: str


@dataclass
class VehicleStatus:
    soc: int
    energy_capacity: float
    ac_vehicle_ready: str
    dc_current_request: float
    dc_present_current: float
    dc_contactors_closed: str
    dc_normal_end_of_charge: str
    dc_battery_voltage: float
    dc_inlet_voltage: float


enable_can_log = False
logged_messages = {}
log_filename = f"./pev_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log"


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


class AdvanticsPEVInterfaceV2(can.Listener):
    def __init__(self, app: Application) -> None:
        self._app = app
        self._bus = app.bus
        self._db: cantools.database.Database = cantools.database.load_file(
            'Advantics_Generic_PEV_protocol_v2.kcd',
        )  # type: ignore[reportAttributeAccessIssue]

        self._bus.set_filters(
            [
                can.typechecking.CanFilterExtended(
                    can_id=message.frame_id,
                    can_mask=0xFFF,
                    extended=False,
                )
                for message in self._db.messages
            ],
        )

        self.vehicle_control = VehicleControl(
            ac_evse_is_ready='----',
            dc_close_contactors='----',
            ###
            dc_contactor_pos_fb='----',
            dc_contactor_neg_fb='----',
            digital_inputs='----',
            stop_charge='----',
            ptc0=-40,
            ptc1=-40,
            ptc2=-40,
            cpu_temp=-40,
            can_sensor_temp=-40,
        )
        self._app.update_vehicle_control(self.vehicle_control)

        self.charger_status = ChargerStatus(
            max_current=0,
            rcd_status='----',
        )
        self._app.update_charger_status(self.charger_status)

        self.vehicle_parameters = VehicleParameters(
            target_energy_request=0,
            min_energy_request=0,
            max_energy_request=0,
            min_v2x_energy_request=0,
            max_v2x_energy_request=0,
            departure_time=0,
        )
        self._app.update_vehicle_parameters(self.vehicle_parameters)

        self.session_status = SessionStatus(
            communication_stage='----',
            protocol='----',
            pins='----',
            cp_duty_cycle=0,
            cp_top_voltage=0,
            cp_state='----',
            pp_resistance=0,
            inlet_lock_state='----',
        )
        self._app.update_session_status(self.session_status)

        self.vehicle_status = VehicleStatus(
            soc=0,
            energy_capacity=0,
            ac_vehicle_ready='----',
            dc_current_request=0,
            dc_present_current=0,
            dc_contactors_closed='----',
            dc_normal_end_of_charge='----',
            dc_battery_voltage=0,
            dc_inlet_voltage=0,
        )
        self._app.update_vehicle_status(self.vehicle_status)

    def on_message_received(self, msg: can.Message) -> None:
        try:
            message = self._db.get_message_by_frame_id(msg.arbitration_id)
        except KeyError:
            return
        signals = message.decode(msg.data)

        log_can_msg(message.name, signals, message.senders)

        # Messages sent by the controller

        if message.name == 'EVSE_Information':
            self.session_status.communication_stage = str(signals['Communication_Stage'])
            self.session_status.protocol = str(signals['Protocol'])
            self.session_status.pins = str(signals['Pins'])
            self._app.update_session_status(self.session_status)
            self.charger_status.max_current = float(signals['Max_Current'])
            self.charger_status.rcd_status = str(signals['RCD_Status'])
            self._app.update_charger_status(self.charger_status)

        elif message.name == 'AC_Control':
            self.vehicle_control.ac_evse_is_ready = str(signals['Ready_To_Deliver_Power'])
            self._app.update_vehicle_control(self.vehicle_control)

        elif message.name == 'DC_Control':
            self.vehicle_control.dc_close_contactors = str(signals['Close_Contactors'])
            self._app.update_vehicle_control(self.vehicle_control)

        elif message.name == 'CCS_Extra_Information':
            self.session_status.cp_duty_cycle = int(signals['CP_Duty_Cycle'])
            self.session_status.cp_top_voltage = float(signals['CP_Top_Voltage'])
            self.session_status.cp_state = str(signals['CP_State'])
            self.session_status.pp_resistance = int(signals['PP_Resistance'])
            self.session_status.inlet_lock_state = str(signals['Inlet_Lock_State'])
            self._app.update_session_status(self.session_status)

        elif message.name == 'ADM_CS_EVCC_Inputs':
            self.vehicle_control.dc_contactor_pos_fb = str(
                signals['DC_Contactor_Positive_Feedback']
            )
            self.vehicle_control.dc_contactor_neg_fb = str(
                signals['DC_Contactor_Negative_Feedback']
            )
            self.vehicle_control.digital_inputs = (
                f'1:{"H" if signals['Digital_Input1'] else "L"} '
                f'2:{"H" if signals['Digital_Input2'] else "L"}'
            )
            self.vehicle_control.stop_charge = str(signals['Stop_Charge'])
            self.vehicle_control.ptc0 = int(signals['PTC0'])
            self.vehicle_control.ptc1 = int(signals['PTC1'])
            self.vehicle_control.ptc2 = int(signals['PTC2'])
            self.vehicle_control.cpu_temp = int(signals['CPU_Temperature'])
            self.vehicle_control.can_sensor_temp = int(signals['CAN_Sensor_Temperature'])
            self._app.update_vehicle_control(self.vehicle_control)

        # Do also the messages going to the controller
        # in case we are just watching someone else session

        elif message.name == 'EV_Information':
            self.vehicle_status.soc = int(signals['State_of_Charge'])
            self.vehicle_status.energy_capacity = float(signals['Energy_Capacity'])
            self._app.update_vehicle_status(self.vehicle_status)

        elif message.name == 'AC_Status':
            self.vehicle_status.ac_vehicle_ready = str(signals['Ready_To_Charge'])
            self._app.update_vehicle_status(self.vehicle_status)

        elif message.name == 'DC_Status1':
            self.vehicle_status.dc_current_request = float(signals['Current_Request'])
            self.vehicle_status.dc_present_current = float(signals['Present_Current'])
            self._app.update_vehicle_status(self.vehicle_status)

        elif message.name == 'DC_Status2':
            self.vehicle_status.dc_contactors_closed = str(signals['Contactors_Closed'])
            self.vehicle_status.dc_normal_end_of_charge = str(signals['Normal_End_of_Charge'])
            self.vehicle_status.dc_battery_voltage = float(signals['Battery_Voltage'])
            self.vehicle_status.dc_inlet_voltage = float(signals['Inlet_Voltage'])
            self._app.update_vehicle_status(self.vehicle_status)

        elif message.name == 'EV_Energy_Request':
            self.vehicle_parameters.target_energy_request = float(signals['Target_Energy_Request'])
            self.vehicle_parameters.min_energy_request = float(signals['Minimum_Energy_Request'])
            self.vehicle_parameters.max_energy_request = float(signals['Maximum_Energy_Request'])
            self._app.update_vehicle_parameters(self.vehicle_parameters)

        elif message.name == 'EV_V2X_Energy_Request':
            self.vehicle_parameters.min_v2x_energy_request = float(
                signals['Minimum_V2X_Energy_Request']
            )
            self.vehicle_parameters.max_v2x_energy_request = float(
                signals['Maximum_V2X_Energy_Request']
            )
            self._app.update_vehicle_parameters(self.vehicle_parameters)

        elif message.name == 'EV_Extra_BPT_Information':
            self.vehicle_parameters.departure_time = int(signals['Departure_Time'])
            self._app.update_vehicle_parameters(self.vehicle_parameters)

    def on_error(self, exc: Exception) -> None:
        self._app.live.stop()
        raise exc


class RenderableConsole(Console):
    def __init__(self) -> None:
        super().__init__(record=True, file=open(os.devnull, 'w'))  # noqa: PTH123, SIM115

    def __rich_console__(self, console: Console, options: ConsoleOptions) -> RenderResult:
        texts = self.export_text(clear=False).split('\n')
        yield from texts[-options.height:]


class Application:
    def __init__(self, bus_config: can.typechecking.BusConfig, **interface_config: Any) -> None:
        self._bus_config = bus_config
        self._interface_config = interface_config
        self._bus: can.BusABC | None = None
        self._notifier: can.Notifier | None = None
        self._pev: AdvanticsPEVInterfaceV2 | None = None

        self.console = RenderableConsole()
        self.layout = self._make_layout()
        self.live = Live(self.layout, screen=True, redirect_stderr=True)

    def _make_layout(self) -> Layout:
        layout = Layout(name='EVSE')
        layout.split_column(
            Layout(name='First'),
            Layout(name='Second'),
            Layout(name='Monitor', visible=False),
        )
        layout['First'].split_row(
            Layout(name='vehicle-control'),
            Layout(name='charge-session'),
            Layout(name='vehicle-status'),
        )
        layout['Second'].split_row(
            Layout(name='charger-status'),
            Layout(name='vehicle-parameters'),
            Layout(Panel(self.console, title='Console'), name='placeholder1'),
        )

        return layout

    @property
    def bus(self) -> can.BusABC:
        if (bus := self._bus) is None:
            bus = self._bus = can.Bus(**self._bus_config)

        return bus

    @property
    def pev(self) -> AdvanticsPEVInterfaceV2:
        if (pev := self._pev) is None:
            pev = self._pev = AdvanticsPEVInterfaceV2(self, **self._interface_config)

        return pev

    @property
    def notifier(self) -> can.Notifier:
        if (notifier := self._notifier) is None:
            notifier = self._notifier = can.Notifier(self.bus, [self.pev])

        return notifier

    def start(self) -> None:
        _ = self.notifier  # Implicitly instantiate bus, pev, and notifier, which starts-up

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
            self.live._refresh_thread.join()

    def update_vehicle_control(self, data: VehicleControl) -> None:
        table = Table.grid(expand=True)
        table.box = box.HORIZONTALS
        table.add_column()
        table.add_column()
        table.add_row('[b]DC close contactors:[/]', data.dc_close_contactors)
        table.add_section()
        table.add_row('[b]AC EVSE ready:[/]', self._color_flag(data.ac_evse_is_ready))
        table.add_section()
        table.add_row('[b]DC contactor + feedback:[/]', data.dc_contactor_pos_fb)
        table.add_row('[b]DC contactor - feedback:[/]', data.dc_contactor_neg_fb)
        table.add_row(
            '[b]Stop charge input:[/]',
            self._color_flag(data.stop_charge, not_prefix='No_', invert=True),
        )
        table.add_row('[b]Digital inputs:[/]', self._color_logic(data.digital_inputs))
        table.add_section()
        table.add_row('[b]PTC 0:[/]', f'{data.ptc0} °C')
        table.add_row('[b]PTC 1:[/]', f'{data.ptc1} °C')
        table.add_row('[b]PTC 2:[/]', f'{data.ptc2} °C')
        table.add_row('[b]CPU temperature:[/]', f'{data.cpu_temp} °C')
        table.add_row('[b]CAN sensor temperature:[/]', f'{data.can_sensor_temp} °C')
        self.layout['First']['vehicle-control'].update(Panel(table, title='Vehicle Control'))

    def update_charger_status(self, data: ChargerStatus) -> None:
        table = Table.grid(expand=True)
        table.box = box.HORIZONTALS
        table.add_column()
        table.add_column()
        table.add_row('[b]Max current:[/]', f'{data.max_current:.2f} A')
        table.add_row(
            '[b]RCD status:[/]',
            self._color_flag(data.rcd_status, not_prefix='No_', invert=True),
        )
        self.layout['Second']['charger-status'].update(Panel(table, title='Charger Status'))

    def update_vehicle_parameters(self, data: VehicleParameters) -> None:
        table = Table.grid(expand=True)
        table.box = box.HORIZONTALS
        table.add_column()
        table.add_column()
        table.add_row('[b]Target energy request:[/]', f'{data.target_energy_request:.2f} kWh')
        table.add_row('[b]Minimum energy request:[/]', f'{data.min_energy_request:.2f} kWh')
        table.add_row('[b]Maximum energy request:[/]', f'{data.max_energy_request:.2f} kWh')
        table.add_section()
        table.add_row('[b]Minimum V2X energy request:[/]', f'{data.min_v2x_energy_request:.2f} kWh')
        table.add_row('[b]Maximum V2X energy request:[/]', f'{data.max_v2x_energy_request:.2f} kWh')
        table.add_section()
        table.add_row('[b]Departure time:[/]', f'{data.departure_time} s')
        self.layout['Second']['vehicle-parameters'].update(Panel(table, title='Vehicle Parameters'))

    def update_session_status(self, data: SessionStatus) -> None:
        table = Table.grid(expand=True)
        table.box = box.HORIZONTALS
        table.add_column()
        table.add_column()
        table.add_row('[b]Communication stage:[/]', data.communication_stage)
        table.add_row('[b]Protocol:[/]', data.protocol)
        table.add_row('[b]Pins:[/]', data.pins)
        table.add_section()
        table.add_row('[b]CP duty cycle:[/]', f'{data.cp_duty_cycle} %')
        table.add_row('[b]CP top voltage:[/]', f'{data.cp_top_voltage:0.1f} V')
        table.add_row('[b]CP state:[/]', data.cp_state)
        table.add_row('[b]PP resistance:[/]', f'{data.pp_resistance} Ω')
        table.add_row('[b]Inlet lock state:[/]', data.inlet_lock_state)
        self.layout['First']['charge-session'].update(Panel(table, title='Charge Session'))

    def update_vehicle_status(self, data: VehicleStatus) -> None:
        table = Table.grid(expand=True)
        table.box = box.HORIZONTALS
        table.add_column()
        table.add_column()
        table.add_row('[b]State of charge:[/]', f'{data.soc} %')
        table.add_row('[b]Energy capacity:[/]', f'{data.energy_capacity:0.2f} kWh')
        table.add_section()
        table.add_row('[b]DC inlet voltage:[/]', f'{data.dc_inlet_voltage:0.2f} V')
        table.add_row('[b]DC battery voltage:[/]', f'{data.dc_battery_voltage:0.2f} V')
        table.add_row('[b]DC current request:[/]', f'{data.dc_current_request:0.2f} A')
        table.add_row('[b]DC present current:[/]', f'{data.dc_present_current:0.2f} A')
        table.add_row('[b]DC contactors closed:[/]', data.dc_contactors_closed)
        table.add_row(
            '[b]DC normal end of charge:[/]',
            self._color_flag(data.dc_normal_end_of_charge, not_prefix='No_', invert=True),
        )
        table.add_section()
        table.add_row('[b]AC vehicle ready:[/]', self._color_flag(data.ac_vehicle_ready))
        self.layout['First']['vehicle-status'].update(Panel(table, title='Vehicle Status'))

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
        enable_can_logging: bool = False
) -> None:
    global enable_can_log
    enable_can_log = enable_can_logging
    try:
        bus_config = can.util.load_config(path=can_config)
    except can.exceptions.CanInterfaceNotImplementedError as ex:
        print(f'[red]ERROR:[/] Incorrect CAN configuration. {ex}.')
        raise typer.Abort from ex

    with Application(bus_config) as app:
        app.display()


if __name__ == '__main__':
    typer.run(cli_main)
