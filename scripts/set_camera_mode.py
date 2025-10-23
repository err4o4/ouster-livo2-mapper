#!/usr/bin/env python3
import argparse
import sys
import subprocess
import usb.core
import usb.util

# python3 set_camera_mode.py --trigger 1 --device /dev/video0 --wb 4500 --exposure 2000

def run(cmd):
    # Safer than shell=True; raises on failure
    subprocess.run(cmd, check=True)

def set_v4l2_for_trigger(device: str, trigger: int, wb_temp: int, exposure: int):
    if trigger == 1:
        # External trigger → lock exposure & WB
        run(["v4l2-ctl", "-d", device, "--set-ctrl=exposure_auto=1"])
        run(["v4l2-ctl", "-d", device, "--set-ctrl=white_balance_temperature_auto=0"])
        run(["v4l2-ctl", "-d", device, f"--set-ctrl=white_balance_temperature={wb_temp}"])
        run(["v4l2-ctl", "-d", device, f"--set-ctrl=exposure_absolute={exposure}"])
    else:
        # Master mode → revert to auto
        run(["v4l2-ctl", "-d", device, "--set-ctrl=exposure_auto=0"])
        run(["v4l2-ctl", "-d", device, "--set-ctrl=white_balance_temperature_auto=1"])

def show_current(device: str):
    try:
        out = subprocess.check_output(
            ["v4l2-ctl", "-d", device, "--get-ctrl=exposure_auto,exposure_absolute,white_balance_temperature_auto,white_balance_temperature"],
            text=True
        )
        print("Current controls:")
        print(out)
    except Exception as e:
        print(f"(warn) Could not read current controls: {e}")

def main():
    p = argparse.ArgumentParser(description="Set trigger mode and camera controls.")
    p.add_argument("--trigger", "-t", type=int, choices=(0, 1), required=True,
                   help="0 = master (auto), 1 = external (manual)")
    p.add_argument("--device", "-d", default="/dev/video0",
                   help="V4L2 device path (default: /dev/video0)")
    p.add_argument("--wb", type=int, default=4500,
                   help="White balance temperature when manual (default: 4500)")
    p.add_argument("--exposure", type=int, default=300,
                   help="Exposure absolute when manual (default: 300)")
    p.add_argument("--vendor", type=lambda x: int(x, 0), default=0x2560,
                   help="USB vendor id (hex or int). Default: 0x2560")
    p.add_argument("--product", type=lambda x: int(x, 0), default=0xc128,
                   help="USB product id (hex or int). Default: 0xc128")
    args = p.parse_args()

    # --- Find and prep USB device ---
    dev = usb.core.find(idVendor=args.vendor, idProduct=args.product)
    if dev is None:
        sys.exit("Device not found")

    try:
        i = dev[0].interfaces()[2].bInterfaceNumber
        cfg = dev.get_active_configuration()
        _ = cfg[(2, 0)]
    except Exception as e:
        sys.exit(f"USB interface error: {e}")

    if dev.is_kernel_driver_active(i):
        try:
            dev.detach_kernel_driver(i)
        except usb.core.USBError as e:
            sys.exit(f"Could not detach kernel driver from interface({i}): {e}")

    # --- Send trigger mode control message ---
    msg = [0] * 64
    msg[0] = 0xA8
    msg[1] = 0x1C
    msg[2] = 1 if args.trigger == 1 else 0  # 1=ext trigger, 0=master
    msg[3] = 0x00

    try:
        dev.write(0x06, msg, 1000)
        print(f"Trigger mode set to: {'External (manual)' if args.trigger == 1 else 'Master (auto)'}")
    except usb.core.USBError as e:
        sys.exit(f"USB write failed: {e}")

    # --- Apply v4l2 controls based on trigger mode ---
    try:
        set_v4l2_for_trigger(args.device, args.trigger, args.wb, args.exposure)
    except subprocess.CalledProcessError as e:
        sys.exit(f"v4l2-ctl failed: {e}")

    show_current(args.device)
    print("Done.")

if __name__ == "__main__":
    main()
