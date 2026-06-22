#!/usr/bin/env python3
"""altitude_land_smoke.py — проверка ручного режима (Aleks 2026-06-11):
ползунок высоты вверх/вниз (в hover, без горизонт-команды) + посадка.

НЕ управляет взлётом — он у manual_fly_node (--auto-takeoff). Этот скрипт:
подписан на odom, шлёт /drone/set_altitude и /drone/land, проверяет что дрон
выходит на каждую высоту (|z−target|≤TOL) И держит x,y (дрейф ≤ DRIFT_MAX),
и что посадка опускает на землю. PASS/FAIL + z-трек CSV.

Предусловие: стек поднят + `manual_fly_node --auto-takeoff --alt 2.0` дал READY.
Запуск: python3 help_scripts/altitude_land_smoke.py --csv /tmp/alt_smoke.csv
"""
from __future__ import annotations

import argparse
import csv
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64

TOL = 0.15          # |z − target| ≤ → высота достигнута
DRIFT_MAX = 0.40    # макс дрейф x,y от точки старта манёвра (м) — держит позицию
SETTLE_S = 14.0     # макс время на выход на высоту
LAND_Z = 0.30       # z ниже → приземлился


class Smoke(Node):
    def __init__(self) -> None:
        super().__init__("altitude_land_smoke")
        self.x = self.y = self.z = None
        self.create_subscription(Odometry, "/mavros/local_position/odom",
                                 self._odom, qos_profile_sensor_data)
        self._alt = self.create_publisher(Float64, "/drone/set_altitude", 10)
        self._land = self.create_publisher(Bool, "/drone/land", 10)
        self._rows = []
        self._t0 = None

    def _odom(self, m: Odometry) -> None:
        p = m.pose.pose.position
        self.x, self.y, self.z = p.x, p.y, p.z
        if self._t0 is None:
            self._t0 = time.monotonic()
        self._rows.append((round(time.monotonic() - self._t0, 2),
                           round(p.x, 3), round(p.y, 3), round(p.z, 3)))

    def _spin(self, dt: float) -> None:
        t = time.monotonic()
        while time.monotonic() - t < dt:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_odom(self, timeout: float = 20.0) -> bool:
        t = time.monotonic()
        while self.z is None and time.monotonic() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.z is not None

    def goto_alt(self, target: float) -> tuple[bool, float, float]:
        """Шлёт set_altitude, ждёт выхода. Возвращает (ok, achieved_z, max_drift)."""
        x0, y0 = self.x, self.y
        self._alt.publish(Float64(data=float(target)))
        t = time.monotonic()
        drift = 0.0
        while time.monotonic() - t < SETTLE_S:
            rclpy.spin_once(self, timeout_sec=0.05)
            drift = max(drift, ((self.x - x0) ** 2 + (self.y - y0) ** 2) ** 0.5)
            if abs(self.z - target) <= TOL:
                # подержим ещё чуть, убедимся что держит
                self._spin(2.0)
                drift = max(drift, ((self.x - x0) ** 2 + (self.y - y0) ** 2) ** 0.5)
                return True, self.z, drift
        return False, self.z, drift

    def do_land(self) -> tuple[bool, float]:
        self._land.publish(Bool(data=True))
        t = time.monotonic()
        while time.monotonic() - t < 30.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.z < LAND_Z:
                return True, self.z
        return False, self.z


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="/tmp/alt_land_smoke.csv")
    ap.add_argument("--seq", default="1.0,2.0,0.6",
                    help="последовательность высот ползунка (м)")
    args = ap.parse_args()

    rclpy.init()
    s = Smoke()
    if not s.wait_odom():
        print("FAIL: нет odom (стек не готов?)")
        return 2
    s._spin(1.0)
    print(f"старт: z={s.z:.2f}м (ожидаем hover после взлёта ~2.0)")

    results = []
    ok_all = True
    for tgt in [float(v) for v in args.seq.split(",")]:
        ok, z, drift = s.goto_alt(tgt)
        flag = "PASS" if (ok and drift <= DRIFT_MAX) else "FAIL"
        if flag == "FAIL":
            ok_all = False
        print(f"  set_altitude {tgt:.2f} → z={z:.2f}м drift={drift:.2f}м "
              f"[{'дост.' if ok else 'НЕ дост.'} / {'держит' if drift <= DRIFT_MAX else 'ДРЕЙФ'}] {flag}")
        results.append((tgt, z, drift, flag))

    okl, zl = s.do_land()
    lflag = "PASS" if okl else "FAIL"
    if not okl:
        ok_all = False
    print(f"  land → z={zl:.2f}м [{lflag}]")

    with open(args.csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z"])
        w.writerows(s._rows)
    print(f"z-трек: {args.csv} ({len(s._rows)} точек)")

    print("=== РЕЗУЛЬТАТ:", "✅ PASS (высота вверх/вниз + посадка работают)"
          if ok_all else "❌ FAIL", "===")
    s.destroy_node()
    rclpy.shutdown()
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
