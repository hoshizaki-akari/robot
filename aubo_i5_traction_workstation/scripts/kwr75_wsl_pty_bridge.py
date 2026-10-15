#!/usr/bin/env python3
"""Expose a localhost Windows COM4 byte stream as a WSL pseudo-serial port."""

import argparse
import errno
import os
import selectors
import socket
import tty
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18755)
    parser.add_argument("--link", type=Path, default=Path("/tmp/kwr75d-bridge-tty"))
    args = parser.parse_args()
    if args.link.is_symlink():
        previous_target = os.readlink(args.link)
        if previous_target.startswith("/dev/pts/") and not os.path.exists(previous_target):
            args.link.unlink()
    if args.link.exists() or args.link.is_symlink():
        raise SystemExit(f"Refusing to overwrite existing path: {args.link}")
    with socket.create_connection((args.host, args.port), timeout=5) as sock:
        sock.settimeout(None)
        master, slave = os.openpty()
        tty.setraw(slave)
        slave_name = os.ttyname(slave)
        os.symlink(slave_name, args.link)
        print(f"PTY_READY={args.link} -> {slave_name}", flush=True)
        selector = selectors.DefaultSelector()
        selector.register(sock, selectors.EVENT_READ)
        selector.register(master, selectors.EVENT_READ)
        try:
            while True:
                for key, _ in selector.select(timeout=0.1):
                    if key.fileobj is sock:
                        data = sock.recv(4096)
                        if not data:
                            return
                        os.write(master, data)
                    else:
                        try:
                            data = os.read(master, 4096)
                        except OSError as error:
                            if error.errno == errno.EIO:
                                continue
                            raise
                        if data:
                            sock.sendall(data)
        finally:
            selector.close()
            os.close(master)
            os.close(slave)
            if args.link.is_symlink() and os.readlink(args.link) == slave_name:
                args.link.unlink()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
