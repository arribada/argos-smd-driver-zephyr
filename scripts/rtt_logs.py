#!/usr/bin/env python3
"""RTT Log viewer - connects to JLink RTT server and displays logs."""

import socket
import sys
import time

def main():
    host = 'localhost'
    port = 19021

    print(f"Connecting to RTT server at {host}:{port}...")

    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((host, port))
            print("=== RTT Logs Connected ===")
            print()

            while True:
                data = sock.recv(4096)
                if data:
                    print(data.decode('utf-8', errors='replace'), end='', flush=True)
                else:
                    break

        except ConnectionRefusedError:
            print("Waiting for JLink GDB server...")
            time.sleep(1)
        except KeyboardInterrupt:
            print("\n=== RTT Logs Stopped ===")
            break
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(1)
        finally:
            try:
                sock.close()
            except:
                pass

if __name__ == "__main__":
    main()
