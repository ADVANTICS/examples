from advmonitors.evse import maindef cli_main(can_config: str = "can.conf", enable_can_logging: bool = False) -> None:
    global enable_can_log
    enable_can_log = enable_can_logging
    can_config_path = resources.files("advmonitors") / "conf" / can_config
    try:
        bus_config = can.util.load_config(path=can_config_path)
    except can.exceptions.CanInterfaceNotImplementedError as ex:
        print(f'[red]ERROR:[/] Incorrect CAN configuration. {ex}.')
        raise typer.Abort from ex

    with Application(bus_config) as app:
        app.display()


def main():
    typer.run(cli_main)


if __name__ == '__main__':
    main()


main()
