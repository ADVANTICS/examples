"""
Standalone script to run EVSE simulation locally without
installing the package
"""
import typer
from advsimulators.evse.generic_v3 import cli_main

typer.run(cli_main)
