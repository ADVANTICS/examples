"""
Standalone script to run PEV simulation locally without
installing the package
"""
from advsimulators.pev.generic_v1 import cli_main
import typer

typer.run(cli_main)
