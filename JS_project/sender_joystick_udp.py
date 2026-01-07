#!/usr/bin/env python3
# sender_joystick_udp_debug.py
import socket, json, time, argparse, pygame, math, traceback, sys

def ewma(prev, cur, alpha=0.25):
    return cur if prev is None else alpha*cur + (1-alpha)*prev

def apply_deadzone(v, dz=0.08):
    if abs(v) < dz: return 0.0
    s = (abs(v)-dz)/(1-dz); s=max(0.0,min(1.0,s))
    return math.copysign(s, v)

def open_joystick(idx):
    try:
        if pygame.joystick.get_count() <= idx:
            print(f"[ERR] No joystick at index {idx}. Detected={pygame.joystick.get_count()}")
            return None
        j = pygame.joystick.Joystick(idx)
        j.init()
        print(f"[INFO] Using joystick [{idx}]: {j.get_name()} | axes={j.get_numaxes()} buttons={j.get_numbuttons()} hats={j.get_numhats()}")
        return j
    except Exception:
        print("[EXC] open_joystick failed:\n" + traceback.format_exc())
        return None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="140.113.148.99")
    ap.add_argument("--port", type=int, default=9999)
    ap.add_argument("--hz", type=int, default=60)
    ap.add_argument("--deadzone", type=float, default=0.12)
    ap.add_argument("--alpha", type=float, default=1.0)
    ap.add_argument("--joy", type=int, default=0)
    ap.add_argument("--broadcast", action="store_true", help="Enable UDP broadcast socket option")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    try:
        pygame.init()
        pygame.joystick.init()
    except Exception:
        print("[EXC] pygame init failed:\n" + traceback.format_exc())
        input("Press Enter to exit...")
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if args.broadcast:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    target = (args.ip, args.port)

    j = open_joystick(args.joy)
    last_axes=None; last_btns=None
    interval = 1.0/max(1,args.hz)
    last_send=0.0; last_print=0.0

    print(f"[INFO] Target {target}  |  Hz={args.hz}  deadzone={args.deadzone}  alpha={args.alpha}")
    print("[HINT] If you see crashes, the error will print below instead of closing the window.\n")

    while True:
        try:
            # 保持事件循環
            pygame.event.pump()
            for _ in pygame.event.get():
                pass
            if j is None or not j.get_init():
                time.sleep(0.5)
                j = open_joystick(args.joy)
                continue

            now = time.time()
            if now - last_send >= interval:
                axes=[]
                for i in range(j.get_numaxes()):
                    v = apply_deadzone(j.get_axis(i), args.deadzone)
                    v = ewma(last_axes[i] if (last_axes and i<len(last_axes)) else None, v, args.alpha)
                    axes.append(v)
                btns=[int(j.get_button(i)) for i in range(j.get_numbuttons())]
                hats=[j.get_hat(i) for i in range(j.get_numhats())]

                payload = {
                    "ts_unix_ms": int(now*1000),
                    "device": j.get_name(),
                    "index": args.joy,
                    "axes": axes,
                    "buttons": btns,
                    "hats": hats,
                    "meta": {"hz":args.hz,"deadzone":args.deadzone,"alpha":args.alpha}
                }
                data = json.dumps(payload).encode("utf-8")

                try:
                    sock.sendto(data, target)
                except OSError as e:
                    # 廣播或網路相關錯誤常見：10013/10051/10022
                    print(f"[SOCK ERR] {e}  ip={args.ip} port={args.port}  (Tip: use --broadcast for 255.255.255.255)")
                last_send = now

                if args.verbose:
                    if last_btns is None or btns != last_btns:
                        print(f"[BTN] {btns}")
                    if last_axes is None or any(abs(a-b)>0.02 for a,b in zip(axes, last_axes)):
                        if now - last_print > 0.2:
                            print(f"[AX] {['%.2f'%a for a in axes]}  HAT={hats}")
                            last_print = now
                last_axes = axes; last_btns = btns

            time.sleep(0.001)

        except KeyboardInterrupt:
            print("\n[INFO] Stopped by user.")
            break
        except Exception:
            # 不讓程式關閉，直接印出錯誤並繼續
            print("[EXC] Loop exception:\n" + traceback.format_exc())
            time.sleep(0.5)

if __name__ == "__main__":
    try:
        main()
    except Exception:
        print("[EXC] Top-level exception:\n" + traceback.format_exc())
        input("Press Enter to exit...")
