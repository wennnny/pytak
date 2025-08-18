#! /usr/bin/env python3
import asyncio, time, csv, os
from collections import defaultdict
from statistics import mean

def _percentile(arr, p):
    if not arr:
        return None
    arr2 = sorted(arr)
    k = int(round((p / 100) * (len(arr2) - 1)))
    return arr2[k]

class PerSecondTxLogger:
    """
    透過 asyncio.Queue 接收事件，寫兩個檔案：
      - tak_tx_log.csv：逐筆訊息（含 latency_ms）
      - tak_tx_summary.csv：每秒彙總（GPS latency 的 avg/p50/p95/max）
    """
    def __init__(self, csv_path="tak_tx_log.csv", summary_path="tak_tx_summary.csv"):
        self.csv_path = csv_path
        self.summary_path = summary_path
        self._bucket = defaultdict(list)
        self._init_files()

    def _init_files(self):
        self._log_new = not os.path.exists(self.csv_path)
        self._sum_new = not os.path.exists(self.summary_path)
        self._log_f = open(self.csv_path, "a", newline="", encoding="utf-8")
        self._sum_f = open(self.summary_path, "a", newline="", encoding="utf-8")
        self._log_w = csv.writer(self._log_f)
        self._sum_w = csv.writer(self._sum_f)
        if self._log_new:
            self._log_w.writerow([
                "ts_epoch", "msg_type", "seq", "index",
                "lat", "lon", "x", "y", "z", "latency_ms"
            ])
        if self._sum_new:
            self._sum_w.writerow([
                "second_epoch", "total_msgs", "gps_count",
                "gps_avg_ms", "gps_p50_ms", "gps_p95_ms", "gps_max_ms"
            ])

    async def run(self, q: asyncio.Queue):
        try:
            while True:
                # 1) 盡量取事件；2) 每秒 flush 已完成秒的彙總
                try:
                    evt = await asyncio.wait_for(q.get(), timeout=1.0)
                    self._log_event(evt)
                except asyncio.TimeoutError:
                    pass
                self._flush_completed_seconds()
        finally:
            try:
                self._log_f.close()
                self._sum_f.close()
            except Exception:
                pass

    def _log_event(self, evt: dict):
        t = float(evt.get("t", time.time()))
        sec = int(t)
        self._bucket[sec].append(evt)
        # 逐筆 CSV
        row = [
            int(t),
            evt.get("type"),
            evt.get("seq"),
            evt.get("index"),
            evt.get("lat"),
            evt.get("lon"),
            evt.get("x"),
            evt.get("y"),
            evt.get("z"),
            evt.get("latency_ms"),
        ]
        self._log_w.writerow(row)
        self._log_f.flush()

    def _flush_completed_seconds(self):
        now_sec = int(time.time())
        done_secs = [s for s in self._bucket if s < now_sec]
        for s in sorted(done_secs):
            evts = self._bucket.pop(s)
            total = len(evts)
            gps_lats = [
                e.get("latency_ms") for e in evts
                if e.get("type") == "gps" and isinstance(e.get("latency_ms"), (int, float))
            ]
            gps_lats_nonneg = [x for x in gps_lats if x is not None and x >= 0]

            row = [
                s, total, len(gps_lats),
                round(mean(gps_lats_nonneg), 3) if gps_lats_nonneg else None,
                round(_percentile(gps_lats_nonneg, 50), 3) if gps_lats_nonneg else None,
                round(_percentile(gps_lats_nonneg, 95), 3) if gps_lats_nonneg else None,
                round(max(gps_lats_nonneg), 3) if gps_lats_nonneg else None,
            ]
            self._sum_w.writerow(row)
            self._sum_f.flush()
