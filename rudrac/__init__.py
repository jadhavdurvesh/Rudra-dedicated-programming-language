"""rudrac: compiler for the Rudra slave-node language (.rudra -> Arduino-framework C++)."""
from .check import check
from .gen import generate
from .sim import Sim
from .syntax import parse
from .util import VERSION, RudraError

__all__ = ["check", "generate", "parse", "Sim", "RudraError", "VERSION"]
