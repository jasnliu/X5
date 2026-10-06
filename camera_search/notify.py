"""Send a one-shot supervisor fault into the local camera status socket."""
from __future__ import annotations

import argparse

from .protocol import DetectionSender


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", required=True)
    parser.add_argument("--detail", required=True)
    args = parser.parse_args()
    sender = DetectionSender(args.socket)
    try:
        sender.status("error", args.detail)
    finally:
        sender.close()


if __name__ == "__main__":
    main()
