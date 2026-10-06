"""Send a one-shot supervisor fault to the playback audio socket."""
from __future__ import annotations

import argparse

from .audio import AudioSender


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", required=True)
    parser.add_argument("--detail", required=True)
    args = parser.parse_args()
    sender = AudioSender(args.socket)
    try:
        sender.status("error", args.detail)
    finally:
        sender.close()


if __name__ == "__main__":
    main()
