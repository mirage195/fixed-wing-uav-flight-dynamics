"""Command-line interface for the complete analysis workflow."""

from __future__ import annotations

import argparse
from collections.abc import Sequence

from . import __version__
from .configuration import (
    ConfigurationError,
    default_configuration,
    load_configuration,
)
from .workflow import format_analysis_summary, run_analysis


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="uav-dynamics",
        description="Analyze linear fixed-wing UAV dynamics about a trim point.",
    )
    parser.add_argument(
        "--config",
        metavar="PATH",
        help="TOML aircraft/aerodynamic configuration; defaults to bundled data",
    )
    parser.add_argument(
        "--output-dir",
        default="results",
        help="directory for JSON, CSV, PNG, and SVG artifacts (default: results)",
    )
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="skip PNG and SVG generation",
    )
    parser.add_argument(
        "--show-plots",
        action="store_true",
        help="display figures after saving them",
    )
    parser.add_argument(
        "--version",
        action="version",
        version=f"%(prog)s {__version__}",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the CLI and return a process exit status."""

    parser = build_parser()
    arguments = parser.parse_args(argv)
    if arguments.no_plots and arguments.show_plots:
        parser.error("--no-plots and --show-plots cannot be used together")
    try:
        configuration = (
            load_configuration(arguments.config)
            if arguments.config
            else default_configuration()
        )
        results = run_analysis(
            configuration,
            arguments.output_dir,
            generate_plots=not arguments.no_plots,
            show_plots=arguments.show_plots,
        )
    except (ConfigurationError, OSError, RuntimeError, ValueError) as error:
        parser.exit(2, f"{parser.prog}: error: {error}\n")
    print(format_analysis_summary(results, arguments.output_dir))
    return 0
