#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
猫撃退システム 本番用メインプログラム
===================================================
構成: Raspberry Pi 4
  - PIRセンサー  : 動体検知
  - リレー       : ダイヤフラムポンプ電源のON/OFF（散水）
  - アクティブブザー: 散水直前の警告ビープ
  - USBカメラ    : 検知時・散水時の様子を撮影（任意。無くても動作する）

動作の流れ（ON時）:
  動体検知 → 📷撮影 → 0.5秒の溜め → ビープ1秒 → 1秒待機 → 散水（📷撮影）→ 5秒クールタイム

Webアプリ（Flask）から以下の3状態を制御できます:
  - ON     : 通常稼働
  - 一時OFF : 5分間停止し、その後自動でONに復帰
  - OFF    : このアプリケーション自体を終了

    スマホ等から  http://192.168.x.x/   にアクセスして操作します。
    （※ポート80で待受けするため sudo での起動が必要です。詳細は末尾を参照）
"""

import os
import json
import time
import shutil
import signal
import atexit
import threading
from datetime import datetime, timedelta
from collections import deque

from flask import Flask, jsonify, render_template_string
from gpiozero import MotionSensor, OutputDevice

try:
    import cv2  # USBカメラ用（sudo apt install python3-opencv）。無ければ撮影だけスキップ
except ImportError:
    cv2 = None

# ============================================================
#  設定（ここを変えれば挙動を調整できます）
# ============================================================
# --- ピンアサイン（BCM番号）---
PIR_PIN = 18     # 人感センサー OUT (GPIO 18 / 12番ピン)
RELAY_PIN = 17   # リレーモジュール IN (GPIO 17 / 11番ピン)
BUZZER_PIN = 27  # アクティブブザー I/O (GPIO 27 / 13番ピン)
                 # ※起動直後(アプリ未起動)の誤鳴きを防ぐため、/boot/firmware/config.txt に
                 #   「gpio=27=op,dh」を追記し、起動時からピンをHIGH(=静音)に固定すること。

# --- タイミング（秒）---
PRE_BEEP_DELAY = 0.5     # 検知→ビープまでの「溜め」（この頭で1枚目を撮影）
BEEP_DURATION = 1.0      # ビープを鳴らす長さ
WAIT_AFTER_BEEP = 1.0    # ビープ後、散水までの待機
SPRAY_DURATION = 4.0     # 散水（リレーON）の長さ
COOLDOWN = 5.0           # 散水後のクールタイム（この間は再検知しても無視）
PAUSE_DURATION = 5 * 60  # 「一時OFF」の停止時間（5分）
TEST_PUMP_DURATION = 5.0 # テストポンプ（呼び水・動作確認用）の駆動時間

# --- 安全機構 ---
# 暴走検知：直近RUNAWAY_WINDOW秒で自動散水がRUNAWAY_MAX回以上 → 故障モードにラッチ。
#   （1サイクル最短≒11.5秒＝60秒の窓に入るのは最大6回なので、6回は「その1分ずっと反応し続けた」異常状態）
#   ※手動のテストポンプ（呼び水）はこの回数に含めない。
RUNAWAY_MAX = 6
RUNAWAY_WINDOW = 60
# 最大ON時間ウォッチドッグ：リレーが連続でこの秒数を超えてONなら強制OFF＆故障モード。
#   （"散水が規定秒で切れない"バグ・不具合への別レイヤー保険。実際の上限は下記の通り
#     正規の最長駆動＝散水/テストポンプより十分大きい値に自動調整される）
MAX_PUMP_ON_SEC = 20

# --- ハードウェアの極性 ---
# リレー: 標準はactive_high=True（信号HIGHでON）。
#   ローアクティブ基板なら active_high=False に変更してください。
RELAY_ACTIVE_HIGH = True
# ブザー: ローレベルトリガ（信号LOWで鳴る）なので active_high=False。
#   ★重要: このモジュールは必ず 3.3V で駆動すること（VCC → Piの3.3Vピン）。
#     5Vで駆動すると、GPIOのHIGH(3.3V)ではOFFにしきれず鳴り続ける（電圧ミスマッチ）。
#     3.3V駆動なら「HIGH=完全OFF」となり正常に動作する。
#   active_high=False + initial_value=False で off=HIGH=静音／.on()=LOW=鳴る、となります。
BUZZER_ACTIVE_HIGH = False

# --- USBカメラ（撮影）---
CAMERA_ENABLED = True       # Falseでカメラ機能を丸ごと無効化
CAMERA_DEVICE = 0           # /dev/video0
CAMERA_WIDTH = 640          # 撮影解像度。これより大きい画像が来たら縮小して保存
CAMERA_HEIGHT = 480
PHOTO_JPEG_QUALITY = 70     # JPEG画質(0-100)。640x480・70で1枚およそ30〜80KB
SPRAY_SHOT_DELAY = 0.5      # ポンプON→2枚目を撮るまでの遅れ（ノズルから水が出るまでの時間）
PHOTO_RETENTION_DAYS = 30   # これより古い日付フォルダは自動削除（SD容量の保護）

# --- Webサーバー ---
HOST = "0.0.0.0"   # LAN内のどの端末からもアクセス可能に
PORT = 5000          # ポート5000番に変更

# --- センサー初期化待ち（秒）---
SENSOR_WARMUP = 5

# --- ロギング / 集計 ---
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATS_PATH = os.path.join(BASE_DIR, "stats.json")    # 1日ごとの集計（小さいJSON）
EVENTS_PATH = os.path.join(BASE_DIR, "events.jsonl")  # 生イベントログ（1行1JSON）
PHOTO_DIR = os.path.join(BASE_DIR, "photos")          # 撮影画像（photos/YYYY-MM-DD/*.jpg）
STATS_FLUSH_INTERVAL = 60   # 稼働時間の集計をディスクへ書く間隔（秒）。SD保護のため大きめ
DASHBOARD_DAYS = 14         # ダッシュボードに表示する日数
# CPU温度のしきい値（ダッシュボードの色分け・℃）
TEMP_WARN = 60.0
TEMP_HOT = 70.0


# ============================================================
#  デバイス初期化
# ============================================================
pir = MotionSensor(PIR_PIN)
relay = OutputDevice(RELAY_PIN, active_high=RELAY_ACTIVE_HIGH, initial_value=False)
buzzer = OutputDevice(BUZZER_PIN, active_high=BUZZER_ACTIVE_HIGH, initial_value=False)


# --- リレー操作の一元化（ON時刻を記録し、最大ON時間ウォッチドッグで監視する）---
_relay_lock = threading.Lock()
_relay_on_since = None  # リレーをONにした時刻（monotonic）。OFF中はNone


def relay_on():
    global _relay_on_since
    with _relay_lock:
        relay.on()
        _relay_on_since = time.monotonic()


def relay_off():
    global _relay_on_since
    with _relay_lock:
        relay.off()
        _relay_on_since = None


def relay_on_seconds():
    """リレーが連続ONになっている秒数（OFF中は0）。"""
    with _relay_lock:
        if _relay_on_since is None:
            return 0.0
        return time.monotonic() - _relay_on_since


# ============================================================
#  システム状態
# ============================================================
class SystemState:
    """スレッド間で共有するシステムの状態。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.mode = "ON"            # "ON" または "PAUSED"
        self.pause_until = None     # PAUSED解除予定時刻（datetime）
        self.last_motion = None     # 最後に検知した時刻
        self.last_spray = None      # 最後に散水した時刻
        self.spray_count = 0        # 散水回数（起動後）
        self.busy = False           # ビープ〜散水シーケンス実行中か
        self.logs = deque(maxlen=30)  # 直近のログ
        self.fault = False          # 故障モード（自動散水を停止・ラッチ）
        self.fault_reason = ""      # 故障モードに入った理由
        self.spray_times = deque()  # 自動散水のmonotonic時刻（暴走検知用・手動は含めない）

    def log(self, message):
        stamp = datetime.now().strftime("%H:%M:%S")
        line = f"[{stamp}] {message}"
        with self.lock:
            self.logs.appendleft(line)
        print(line, flush=True)


state = SystemState()
shutdown_event = threading.Event()
PROCESS_START = time.time()  # プロセス起動時刻（稼働時間の算出用）


# ============================================================
#  日次集計（低コスト設計）
#    - 集計はイベント発生時にその場で加算するだけ（閲覧時に再計算しない）
#    - 稼働時間はメモリに貯め、STATS_FLUSH_INTERVAL 秒ごとにまとめ書き
# ============================================================
STAT_KEYS = ("armed_sec", "detections", "sprays", "pauses", "resumes", "offs", "startups")


class DailyStats:
    """日付(YYYY-MM-DD)ごとのカウンタをメモリ保持し、小さなJSONに永続化する。"""

    def __init__(self, path):
        self.path = path
        self.lock = threading.Lock()
        self.data = self._load()
        self._dirty = False

    def _load(self):
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                return json.load(f)
        except (FileNotFoundError, ValueError):
            return {}

    @staticmethod
    def _today():
        return datetime.now().strftime("%Y-%m-%d")

    def _bucket(self):
        """呼び出し時点の日付バケツを返す（無ければ作る）。※lock取得済み前提。"""
        day = self._today()
        b = self.data.get(day)
        if b is None:
            b = {k: 0 for k in STAT_KEYS}
            self.data[day] = b
        return b

    def incr(self, key, n=1):
        with self.lock:
            self._bucket()[key] += n
            self._dirty = True

    def add_armed(self, seconds):
        if seconds <= 0:
            return
        with self.lock:
            self._bucket()["armed_sec"] += seconds
            self._dirty = True

    def flush(self):
        """差分がある時だけ、アトミックに書き込む（SD保護＆破損防止）。"""
        with self.lock:
            if not self._dirty:
                return
            tmp = self.path + ".tmp"
            try:
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump(self.data, f, ensure_ascii=False)
                os.replace(tmp, self.path)
                self._dirty = False
            except OSError as e:
                print(f"[stats] 書き込み失敗: {e}", flush=True)

    def recent(self, days):
        """直近days日分を古い順のリストで返す（表示用・軽量）。"""
        with self.lock:
            out = []
            for i in range(days - 1, -1, -1):
                day = (datetime.now() - timedelta(days=i)).strftime("%Y-%m-%d")
                b = self.data.get(day, {k: 0 for k in STAT_KEYS})
                row = {"date": day}
                row.update({k: b.get(k, 0) for k in STAT_KEYS})
                out.append(row)
            return out


stats = DailyStats(STATS_PATH)


def log_event(event_type, **extra):
    """生イベントを1行JSONで追記（後からの情報収集・分析用）。低頻度なので即時追記。"""
    record = {"ts": datetime.now().isoformat(timespec="seconds"), "type": event_type}
    record.update(extra)
    try:
        with open(EVENTS_PATH, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")
    except OSError as e:
        print(f"[event] 書き込み失敗: {e}", flush=True)


def read_cpu_temp():
    """CPU温度(℃)を返す。読めなければNone。ファイル1個読むだけの激軽処理。"""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return round(int(f.read().strip()) / 1000.0, 1)
    except (OSError, ValueError):
        return None


# ============================================================
#  USBカメラ（撮影）
# ============================================================
class Camera:
    """USBカメラを常時オープンし、撮影要求が来たら「その瞬間の」1枚を保存する。
    - 撮影のたびにopenすると0.5〜1秒かかり露出も合わず暗くなるため、常時オープンにしておく。
    - 裏ではgrab()（デコードなし＝軽い）だけを回してバッファを最新に保ち、
      撮影要求があった時だけretrieve()でデコードする。
    - capture()は要求を積むだけで即戻る。JPEG圧縮・書き込みも別スレッド。
      → カメラが無い／抜けた／SDが遅い場合でも、散水シーケンスのタイミングには影響しない。"""

    RETRY_SEC = 10        # カメラが見つからない時の再接続間隔
    MAX_GRAB_FAILS = 20   # 連続でこの回数grabに失敗したら切断とみなす

    def __init__(self):
        self._lock = threading.Lock()
        self._pending = []      # 撮影待ちの保存先（絶対パス）
        self._running = False   # 今フレームを取得できている状態か

    def start(self):
        if not CAMERA_ENABLED:
            return
        if cv2 is None:
            state.log("📷 OpenCV未導入のため撮影なし（sudo apt install python3-opencv）")
            return
        threading.Thread(target=self._loop, daemon=True).start()

    def capture(self, taken_for, label):
        """撮影を要求する（非ブロッキング）。受け付けたら保存先の相対パス、できなければNone。"""
        day = taken_for.strftime("%Y-%m-%d")
        name = f"{taken_for.strftime('%H%M%S')}_{label}.jpg"
        path = os.path.join(PHOTO_DIR, day, name)
        with self._lock:
            if not self._running:
                return None
            self._pending.append(path)
        return f"photos/{day}/{name}"

    def _open(self):
        cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None
        # MJPGにするとUSB帯域・CPUともに軽い。解像度は控えめに（容量節約）
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _loop(self):
        announced = None  # 直前にログした状態（同じ内容を連投しないため）
        while not shutdown_event.is_set():
            cap = self._open()
            if cap is None:
                if announced != "missing":
                    state.log(f"📷 カメラが見つかりません（/dev/video{CAMERA_DEVICE}）。{self.RETRY_SEC}秒ごとに再試行")
                    announced = "missing"
                shutdown_event.wait(self.RETRY_SEC)
                continue

            fails = 0
            while not shutdown_event.is_set() and fails < self.MAX_GRAB_FAILS:
                if not cap.grab():
                    fails += 1
                    with self._lock:
                        self._running = False
                    time.sleep(0.1)
                    continue
                fails = 0
                with self._lock:
                    self._running = True
                    pending, self._pending = self._pending, []
                if announced != "ready":
                    state.log("📷 カメラ準備OK")
                    announced = "ready"
                if pending:
                    ok, frame = cap.retrieve()
                    if ok:
                        for path in pending:
                            threading.Thread(target=self._save, args=(frame.copy(), path), daemon=True).start()

            with self._lock:
                self._running = False
                self._pending = []
            cap.release()
            if not shutdown_event.is_set():
                state.log("📷 カメラとの接続が切れました。再接続を試みます")
                announced = "lost"
                shutdown_event.wait(1)

    def _save(self, frame, path):
        """縮小（必要なら）＋時刻の焼き込み＋JPEG圧縮して保存。"""
        try:
            h, w = frame.shape[:2]
            if w > CAMERA_WIDTH:
                frame = cv2.resize(frame, (CAMERA_WIDTH, int(h * CAMERA_WIDTH / w)), interpolation=cv2.INTER_AREA)
            stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            for color, thick in (((0, 0, 0), 3), ((255, 255, 255), 1)):  # 縁取りで明暗どちらでも読める
                cv2.putText(frame, stamp, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, thick, cv2.LINE_AA)
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, PHOTO_JPEG_QUALITY])
            if not ok:
                raise OSError("JPEGエンコード失敗")

            day_dir = os.path.dirname(path)
            if not os.path.isdir(day_dir):
                os.makedirs(day_dir, exist_ok=True)
                _cleanup_old_photos()  # 日付が変わった最初の1枚のときだけ掃除
            tmp = path + ".tmp"
            with open(tmp, "wb") as f:
                f.write(buf.tobytes())
            os.replace(tmp, path)  # 書きかけのファイルを残さない
        except Exception as e:
            print(f"[camera] 保存失敗 {path}: {e}", flush=True)


def _cleanup_old_photos():
    """PHOTO_RETENTION_DAYSより古い日付フォルダ（YYYY-MM-DD）を削除する。"""
    limit = datetime.now().date() - timedelta(days=PHOTO_RETENTION_DAYS)
    try:
        entries = os.listdir(PHOTO_DIR)
    except OSError:
        return
    for name in entries:
        try:
            day = datetime.strptime(name, "%Y-%m-%d").date()
        except ValueError:
            continue  # 日付フォルダ以外には触らない
        if day < limit:
            shutil.rmtree(os.path.join(PHOTO_DIR, name), ignore_errors=True)


camera = Camera()


# --- 安全機構：故障モードと最大ON時間ウォッチドッグ ---------------------
def enter_fault(reason):
    """故障モードに入る（自動散水を停止しラッチ）。ポンプ・ブザーも即停止。"""
    with state.lock:
        if state.fault:
            return  # 既に故障モードなら二重には入らない
        state.fault = True
        state.fault_reason = reason
    relay_off()
    buzzer.off()
    state.log(f"🚨 故障モード: {reason}（自動散水を停止。ONで解除）")
    log_event("fault", reason=reason)


def _max_pump_on_cap():
    """最大ON時間の実効上限。正規の最長駆動より必ず大きくする（誤発動防止）。"""
    return max(MAX_PUMP_ON_SEC, SPRAY_DURATION + 3, TEST_PUMP_DURATION + 3)


def watchdog_loop():
    """リレーが実効上限を超えて連続ONなら強制停止＆故障モード。
    ソフトのバグやハングで"切れない"事態への最後のソフト保険。"""
    cap = _max_pump_on_cap()
    while not shutdown_event.is_set():
        if relay_on_seconds() > cap:
            relay_off()
            enter_fault(f"リレーが{cap:.0f}秒を超えて連続ON（強制停止）")
        time.sleep(0.5)


# --- 起動時の復元（ファイルからWebUIの表示を取り戻す）-----------------
EVENT_LABEL = {
    "startup": "起動しました",
    "detect": "★ 動体を検知",
    "spray": "💧 散水",
    "test_pump": "💧 テストポンプ駆動",
    "pause": "🟡 一時OFF",
    "resume": "⏰ 稼働に復帰",
    "on": "🟢 ON",
    "off": "🔴 OFF：終了",
}


def _tail_lines(path, n):
    """ファイル末尾の最大n行を返す。末尾64KBだけ読むのでファイルが育っても軽い。"""
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            block = min(size, 65536)
            f.seek(size - block)
            data = f.read().decode("utf-8", errors="replace")
        return [ln for ln in data.splitlines() if ln.strip()][-n:]
    except (FileNotFoundError, OSError):
        return []


def _format_event_line(ev):
    """イベント辞書 → 黒いログ枠用の1行（履歴は日付付きで当日ログと区別）。"""
    ts = ev.get("ts", "")
    try:
        tstr = datetime.fromisoformat(ts).strftime("%m/%d %H:%M:%S")
    except ValueError:
        tstr = ts
    label = EVENT_LABEL.get(ev.get("type"), ev.get("type", ""))
    return f"[{tstr}] {label}"


def restore_from_log():
    """起動時に events.jsonl から直近ログ・最終検知/散水時刻を復元する。"""
    events = []
    for line in _tail_lines(EVENTS_PATH, 50):
        try:
            events.append(json.loads(line))
        except ValueError:
            continue
    if not events:
        return

    last_motion = None
    last_spray = None
    for ev in events:  # 古い順。更新し続ければ最後（＝最新）の値が残る
        try:
            dt = datetime.fromisoformat(ev.get("ts", ""))
        except ValueError:
            continue
        if ev.get("type") == "detect":
            last_motion = dt
        elif ev.get("type") in ("spray", "test_pump"):
            last_spray = dt

    with state.lock:
        # 古い順に appendleft すると、最新が先頭（index0）に来る＝当日ログと同じ並び
        for ev in events[-30:]:
            state.logs.appendleft(_format_event_line(ev))
        if last_motion:
            state.last_motion = last_motion
        if last_spray:
            state.last_spray = last_spray


# ============================================================
#  散水シーケンス（ビープ→待機→散水）
# ============================================================
def run_spray_sequence():
    """検知時の一連の動作。修正依頼3の順序を厳守。"""
    with state.lock:
        # シーケンス直前に念のため状態を再確認
        # （OFF/一時OFF、故障モード、あるいはテストポンプ駆動中(busy)なら中止）
        if state.mode != "ON" or shutdown_event.is_set() or state.busy or state.fault:
            return
        state.busy = True
        detected_at = datetime.now()
        state.last_motion = detected_at

    try:
        # 1枚目：検知した瞬間（撮影は非ブロッキング。カメラが無ければNone）
        detect_photo = camera.capture(detected_at, "1_detect")
        state.log("★ 動体を検知 → 撮影・警告ビープ" if detect_photo else "★ 動体を検知 → 警告ビープ")
        stats.incr("detections")
        log_event("detect", **({"photo": detect_photo} if detect_photo else {}))
        time.sleep(PRE_BEEP_DELAY)  # 溜め

        buzzer.on()
        time.sleep(BEEP_DURATION)
        buzzer.off()

        time.sleep(WAIT_AFTER_BEEP)

        state.log("💧 散水開始")
        relay_on()
        # 2枚目：水が出始めた瞬間（ポンプONからSPRAY_SHOT_DELAY後）。散水時間の合計は変えない
        shot_delay = min(SPRAY_SHOT_DELAY, SPRAY_DURATION)
        time.sleep(shot_delay)
        spray_photo = camera.capture(detected_at, "2_spray")
        time.sleep(SPRAY_DURATION - shot_delay)
        relay_off()
        state.log("散水終了 → クールタイム")

        with state.lock:
            state.last_spray = datetime.now()
            state.spray_count += 1
        stats.incr("sprays")
        log_event("spray", duration=SPRAY_DURATION, **({"photo": spray_photo} if spray_photo else {}))

        # --- 暴走検知（直近RUNAWAY_WINDOW秒の自動散水回数。手動テストは含めない）---
        now_m = time.monotonic()
        with state.lock:
            state.spray_times.append(now_m)
            while state.spray_times and now_m - state.spray_times[0] > RUNAWAY_WINDOW:
                state.spray_times.popleft()
            count = len(state.spray_times)
        if count >= RUNAWAY_MAX:
            enter_fault(f"直近{RUNAWAY_WINDOW}秒で自動散水{count}回（暴走を検知）")
    finally:
        # どんな経路でも必ず停止させる（安全側）
        buzzer.off()
        relay_off()
        with state.lock:
            state.busy = False


def run_test_pump():
    """テスト/呼び水用にポンプ(リレー)だけを一定時間回す。ビープ・検知とは独立。
    散水シーケンスやテスト同士がぶつからないよう busy で排他制御する。"""
    with state.lock:
        if state.busy or shutdown_event.is_set() or state.fault:
            return  # 既に何か動作中／故障モードなら何もしない
        state.busy = True
    try:
        state.log(f"💧 テストポンプ駆動（{TEST_PUMP_DURATION:.0f}秒・Web操作）")
        log_event("test_pump", duration=TEST_PUMP_DURATION)  # ※散水件数(sprays)・暴走検知には含めない
        relay_on()
        time.sleep(TEST_PUMP_DURATION)
        relay_off()
        state.log("テストポンプ停止")
    finally:
        relay_off()
        with state.lock:
            state.busy = False


# ============================================================
#  監視ループ（バックグラウンドスレッド）
# ============================================================
def monitoring_loop():
    state.log(f"センサー初期化中（約{SENSOR_WARMUP}秒）...")
    time.sleep(SENSOR_WARMUP)
    state.log("監視スタンバイ完了！稼働中です")

    cooldown_until = 0.0  # time.time()基準。この時刻まで再検知しない

    # 稼働時間の集計（メモリに貯めてまとめ書き）
    armed_local = 0.0
    last_tick = time.monotonic()
    last_flush = time.monotonic()

    while not shutdown_event.is_set():
        now_mono = time.monotonic()
        elapsed = now_mono - last_tick
        last_tick = now_mono

        # --- 一時OFFの自動復帰チェック ---
        with state.lock:
            mode = state.mode
            fault = state.fault
            if mode == "PAUSED" and state.pause_until is not None:
                if datetime.now() >= state.pause_until:
                    state.mode = "ON"
                    state.pause_until = None
                    mode = "ON"
                    resumed = True
                else:
                    resumed = False
            else:
                resumed = False
        if resumed:
            state.log("⏰ 一時OFFの時間が経過 → 稼働に復帰しました")
            stats.incr("resumes")
            log_event("resume", reason="auto")

        # --- ON かつ 故障モードでない時間だけ稼働時間として加算 ---
        if mode == "ON" and not fault:
            armed_local += elapsed

        # --- ON・故障でない・クールタイム外なら検知を評価 ---
        if mode == "ON" and not fault and now_mono >= cooldown_until:
            if pir.motion_detected:
                run_spray_sequence()
                cooldown_until = time.monotonic() + COOLDOWN
                last_tick = time.monotonic()  # 散水中の時間は稼働時間に含めない

        # --- 定期フラッシュ（SD保護のためまとめ書き）---
        if now_mono - last_flush >= STATS_FLUSH_INTERVAL:
            stats.add_armed(armed_local)
            armed_local = 0.0
            stats.flush()
            last_flush = now_mono

        time.sleep(0.1)

    # 終了時のクリーンアップ＆集計の確定
    stats.add_armed(armed_local)
    stats.flush()
    buzzer.off()
    relay_off()


# ============================================================
#  Flask Webアプリ
# ============================================================
app = Flask(__name__)

PAGE = """
<!doctype html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>猫撃退システム</title>
  <style>
    :root { color-scheme: light dark; }
    * { box-sizing: border-box; }
    body {
      font-family: -apple-system, "Hiragino Kaku Gothic ProN", "Yu Gothic", sans-serif;
      margin: 0; padding: 24px 16px; background: #0f172a; color: #e2e8f0;
      display: flex; flex-direction: column; align-items: center; min-height: 100vh;
    }
    h1 { font-size: 1.4rem; margin: 0 0 4px; }
    .sub { color: #94a3b8; font-size: .8rem; margin-bottom: 20px; }
    .card {
      background: #1e293b; border-radius: 16px; padding: 20px;
      width: 100%; max-width: 420px; box-shadow: 0 8px 24px rgba(0,0,0,.4);
    }
    .status {
      text-align: center; font-size: 1.5rem; font-weight: 700;
      padding: 16px; border-radius: 12px; margin-bottom: 8px;
    }
    .status small { display:block; font-size:.8rem; font-weight:400; margin-top:6px; color:#cbd5e1; }
    .on     { background: #14532d; color: #4ade80; }
    .paused { background: #713f12; color: #fbbf24; }
    .off    { background: #7f1d1d; color: #f87171; }
    .buttons { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px; margin-top: 16px; }
    button {
      padding: 16px 8px; border: none; border-radius: 12px; font-size: 1rem;
      font-weight: 700; cursor: pointer; color: #fff; transition: transform .05s;
    }
    button:active { transform: scale(.96); }
    .btn-on    { background: #16a34a; }
    .btn-pause { background: #d97706; }
    .btn-off   { background: #dc2626; }
    .btn-test  { background: #0284c7; width: 100%; margin-top: 12px; }
    .test-note { color:#94a3b8; font-size:.72rem; text-align:center; margin-top:6px; }
    .info { margin-top: 18px; font-size: .85rem; color: #cbd5e1; }
    .info div { display: flex; justify-content: space-between; padding: 4px 0; border-bottom: 1px solid #334155; }
    .logs { margin-top: 16px; font-size: .75rem; color: #94a3b8; max-height: 180px; overflow-y: auto; }
    .logs div { padding: 2px 0; font-family: monospace; }
    .foot { color:#64748b; font-size:.7rem; margin-top:18px; text-align:center; }
  </style>
</head>
<body>
  <h1>🐱 猫撃退システム</h1>
  <div class="sub">Raspberry Pi 4 / PIR + リレー + ブザー</div>

  <div class="card">
    <div id="status" class="status on">読み込み中...</div>

    <div class="buttons">
      <button class="btn-on"    onclick="send('on')">ON</button>
      <button class="btn-pause" onclick="send('pause')">一時OFF<br><small>5分停止</small></button>
      <button class="btn-off"   onclick="send('off')">OFF<br><small>アプリ終了</small></button>
    </div>

    <button class="btn-test" id="btn-test" onclick="testPump()">💧 テストポンプ駆動（5秒）</button>
    <div class="test-note">呼び水・動作確認用（ビープ無し／検知とは独立）。連続で回したい時は連打してください</div>

    <div class="info">
      <div><span>最終検知</span><span id="last_motion">-</span></div>
      <div><span>最終散水</span><span id="last_spray">-</span></div>
      <div><span>本日の検知</span><span id="today_detections">-</span></div>
      <div><span>本日の散水</span><span id="today_sprays">-</span></div>
    </div>

    <div class="logs" id="logs"></div>
  </div>
  <div class="foot">画面は2秒ごとに自動更新されます ・ <a href="/dashboard" style="color:#60a5fa;">📊 ダッシュボード</a></div>

<script>
async function refresh() {
  try {
    const r = await fetch('/api/status');
    const s = await r.json();
    const el = document.getElementById('status');
    if (s.fault) {
      el.className = 'status off';
      el.innerHTML = '🚨 故障モード<small>' + (s.fault_reason || '異常を検知') + '<br>安全のため自動散水を停止中。ONを押すと解除します</small>';
    } else if (s.mode === 'ON') {
      el.className = 'status on';
      el.innerHTML = '稼働中 🟢' + (s.busy ? '<small>動作中...</small>' : '<small>監視しています</small>');
    } else {
      el.className = 'status paused';
      el.innerHTML = '一時停止中 🟡<small>あと約 ' + s.pause_remaining + ' で自動復帰</small>';
    }
    document.getElementById('last_motion').textContent = s.last_motion || '-';
    document.getElementById('last_spray').textContent  = s.last_spray  || '-';
    document.getElementById('today_detections').textContent = (s.today_detections || 0) + ' 件';
    document.getElementById('today_sprays').textContent     = (s.today_sprays || 0) + ' 件';
    document.getElementById('logs').innerHTML = s.logs.map(l => '<div>'+l+'</div>').join('');
  } catch (e) {
    document.getElementById('status').innerHTML = '接続できません<small>アプリが終了(OFF)している可能性があります</small>';
    document.getElementById('status').className = 'status off';
  }
}
async function send(action) {
  if (action === 'off') {
    if (!confirm('システム(アプリ)を終了します。再開にはPi側での再起動が必要です。よろしいですか？')) return;
  }
  await fetch('/api/' + action, { method: 'POST' });
  setTimeout(refresh, 300);
}
async function testPump() {
  const btn = document.getElementById('btn-test');
  try {
    const r = await (await fetch('/api/test_pump', { method: 'POST' })).json();
    if (!r.ok) {
      alert(r.reason === 'fault'
        ? '故障モード中は実行できません。先にONを押して解除してください。'
        : '他の動作中のため、いま実行できません。少し待ってからどうぞ。');
      return;
    }
    // 駆動中はボタンを無効化してカウントダウン表示
    let remain = Math.round(r.duration);
    btn.disabled = true; btn.style.opacity = .6;
    const orig = btn.textContent;
    const tick = setInterval(() => {
      btn.textContent = '💧 駆動中... 残り' + remain + '秒';
      if (--remain < 0) {
        clearInterval(tick);
        btn.disabled = false; btn.style.opacity = 1; btn.textContent = orig;
      }
    }, 1000);
  } catch (e) {
    alert('通信に失敗しました。');
  }
  setTimeout(refresh, 300);
}
refresh();
setInterval(refresh, 2000);
</script>
</body>
</html>
"""


def _fmt(dt):
    # 前日以前の復元値も区別できるよう日付付きで返す
    return dt.strftime("%m/%d %H:%M:%S") if dt else None


@app.route("/")
def index():
    return render_template_string(PAGE)


@app.route("/api/status")
def api_status():
    with state.lock:
        remaining = ""
        if state.mode == "PAUSED" and state.pause_until:
            secs = max(0, int((state.pause_until - datetime.now()).total_seconds()))
            remaining = f"{secs // 60}分{secs % 60}秒"
        data = {
            "mode": state.mode,
            "busy": state.busy,
            "fault": state.fault,
            "fault_reason": state.fault_reason,
            "pause_remaining": remaining,
            "last_motion": _fmt(state.last_motion),
            "last_spray": _fmt(state.last_spray),
            "logs": list(state.logs),
        }
    # 「本日の値」は永続化されたstats.jsonから取得（再起動しても消えない）
    today = stats.recent(1)[0]
    data["today_detections"] = today["detections"]
    data["today_sprays"] = today["sprays"]
    return jsonify(data)


@app.route("/api/on", methods=["POST"])
def api_on():
    with state.lock:
        was_paused = state.mode == "PAUSED"
        was_fault = state.fault
        state.mode = "ON"
        state.pause_until = None
        state.fault = False           # 故障モードを解除
        state.fault_reason = ""
        state.spray_times.clear()     # 暴走カウンタもリセット
    state.log("🟢 ONにしました（Web操作）" + ("／故障モードを解除" if was_fault else ""))
    if was_paused:
        stats.incr("resumes")
    log_event("on", source="web", cleared_fault=was_fault)
    return jsonify({"ok": True})


@app.route("/api/pause", methods=["POST"])
def api_pause():
    with state.lock:
        state.mode = "PAUSED"
        state.pause_until = datetime.now() + timedelta(seconds=PAUSE_DURATION)
    state.log(f"🟡 一時OFF（{PAUSE_DURATION // 60}分停止・Web操作）")
    stats.incr("pauses")
    log_event("pause", duration_sec=PAUSE_DURATION)
    return jsonify({"ok": True})


@app.route("/api/off", methods=["POST"])
def api_off():
    state.log("🔴 OFF：アプリケーションを終了します（Web操作）")
    stats.incr("offs")
    log_event("off", source="web")

    def _shutdown():
        # HTTPレスポンスを返す猶予を少し置いてから終了
        time.sleep(0.5)
        shutdown_event.set()
        stats.flush()   # 集計を確実に保存してから落とす
        buzzer.off()
        relay_off()
        os._exit(0)

    threading.Thread(target=_shutdown, daemon=True).start()
    return jsonify({"ok": True})


@app.route("/api/test_pump", methods=["POST"])
def api_test_pump():
    with state.lock:
        if state.fault:
            return jsonify({"ok": False, "reason": "fault"})
        if state.busy:
            return jsonify({"ok": False, "reason": "busy"})
    # バックグラウンドで駆動し、レスポンスは即返す
    threading.Thread(target=run_test_pump, daemon=True).start()
    return jsonify({"ok": True, "duration": TEST_PUMP_DURATION})


# ============================================================
#  ダッシュボード（集計の閲覧）
# ============================================================
def _fmt_duration(seconds):
    """秒 → 「Xh Ym」表記。"""
    seconds = int(seconds)
    h, rem = divmod(seconds, 3600)
    m = rem // 60
    if h > 0:
        return f"{h}時間{m}分"
    return f"{m}分"


@app.route("/api/stats")
def api_stats():
    # 閲覧のたびに再計算はしない。メモリ上の集計をそのまま返すだけ（低コスト）
    rows = stats.recent(DASHBOARD_DAYS)
    for r in rows:
        r["armed_str"] = _fmt_duration(r["armed_sec"])
        r["armed_hours"] = round(r["armed_sec"] / 3600.0, 2)
    today = rows[-1] if rows else None
    return jsonify({
        "days": rows,
        "today": today,
        "cpu_temp": read_cpu_temp(),
        "temp_warn": TEMP_WARN,
        "temp_hot": TEMP_HOT,
        "uptime_str": _fmt_duration(time.time() - PROCESS_START),
        "mode": state.mode,
    })


DASHBOARD = """
<!doctype html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>猫撃退システム ダッシュボード</title>
  <style>
    :root { color-scheme: light dark; }
    * { box-sizing: border-box; }
    body { font-family: -apple-system, "Hiragino Kaku Gothic ProN", "Yu Gothic", sans-serif;
      margin: 0; padding: 20px 14px; background: #0f172a; color: #e2e8f0; }
    .wrap { max-width: 720px; margin: 0 auto; }
    h1 { font-size: 1.3rem; margin: 0 0 2px; }
    .sub { color:#94a3b8; font-size:.8rem; margin-bottom:16px; }
    a { color:#60a5fa; text-decoration:none; }
    .grid { display:grid; grid-template-columns: repeat(auto-fit, minmax(150px,1fr)); gap:12px; margin-bottom:18px; }
    .tile { background:#1e293b; border-radius:14px; padding:16px; }
    .tile .label { color:#94a3b8; font-size:.75rem; }
    .tile .val { font-size:1.6rem; font-weight:700; margin-top:4px; }
    .tile .unit { font-size:.9rem; color:#cbd5e1; font-weight:400; }
    .temp-ok { color:#4ade80; } .temp-warn { color:#fbbf24; } .temp-hot { color:#f87171; }
    .section { background:#1e293b; border-radius:14px; padding:16px; margin-bottom:16px; }
    .section h2 { font-size:.95rem; margin:0 0 12px; color:#e2e8f0; }
    .bars { display:flex; align-items:flex-end; gap:6px; height:140px; padding-top:8px; }
    .bar-col { flex:1; display:flex; flex-direction:column; align-items:center; height:100%; justify-content:flex-end; }
    .bar-stack { width:70%; display:flex; flex-direction:column-reverse; border-radius:4px 4px 0 0; overflow:hidden; }
    .bar-detect { background:#60a5fa; }
    .bar-spray  { background:#f472b6; }
    .bar-date { font-size:.6rem; color:#64748b; margin-top:4px; white-space:nowrap; }
    .bar-num { font-size:.65rem; color:#cbd5e1; margin-bottom:2px; min-height:.8em; }
    .legend { font-size:.72rem; color:#94a3b8; margin-top:8px; }
    .legend span { display:inline-block; width:10px; height:10px; border-radius:2px; margin:0 4px 0 12px; vertical-align:middle; }
    table { width:100%; border-collapse:collapse; font-size:.78rem; }
    th, td { padding:7px 6px; text-align:right; border-bottom:1px solid #334155; white-space:nowrap; }
    th { color:#94a3b8; font-weight:600; }
    td:first-child, th:first-child { text-align:left; }
    .tbl-wrap { overflow-x:auto; }
    .foot { color:#64748b; font-size:.7rem; margin-top:16px; text-align:center; }
  </style>
</head>
<body>
  <div class="wrap">
    <h1>📊 猫撃退システム ダッシュボード</h1>
    <div class="sub"><a href="/">← 操作画面へ戻る</a> ・ 10秒ごとに自動更新</div>

    <div class="grid">
      <div class="tile"><div class="label">本日の稼働時間</div><div class="val" id="t_armed">-</div></div>
      <div class="tile"><div class="label">本日の検知</div><div class="val" id="t_detect">-<span class="unit"> 件</span></div></div>
      <div class="tile"><div class="label">本日の散水</div><div class="val" id="t_spray">-<span class="unit"> 件</span></div></div>
      <div class="tile"><div class="label">本日の一時OFF</div><div class="val" id="t_pause">-<span class="unit"> 回</span></div></div>
      <div class="tile"><div class="label">CPU温度</div><div class="val" id="t_temp">-</div></div>
      <div class="tile"><div class="label">連続稼働</div><div class="val" id="t_uptime" style="font-size:1.2rem;">-</div></div>
    </div>

    <div class="section">
      <h2>日別 検知・散水件数</h2>
      <div class="bars" id="bars"></div>
      <div class="legend"><span style="background:#60a5fa;"></span>検知<span style="background:#f472b6;"></span>散水</div>
    </div>

    <div class="section">
      <h2>日別 明細</h2>
      <div class="tbl-wrap">
        <table>
          <thead><tr><th>日付</th><th>稼働時間</th><th>検知</th><th>散水</th><th>一時OFF</th><th>復帰</th><th>OFF</th><th>起動</th></tr></thead>
          <tbody id="tbody"></tbody>
        </table>
      </div>
    </div>

    <div class="foot">集計は約60秒ごとにディスク保存されます（SDカード保護のため）</div>
  </div>

<script>
function tempClass(t, warn, hot) {
  if (t == null) return '';
  if (t >= hot) return 'temp-hot';
  if (t >= warn) return 'temp-warn';
  return 'temp-ok';
}
async function refresh() {
  try {
    const s = await (await fetch('/api/stats')).json();
    const today = s.today || {};
    document.getElementById('t_armed').textContent  = today.armed_str || '0分';
    document.getElementById('t_detect').innerHTML   = (today.detections||0) + '<span class="unit"> 件</span>';
    document.getElementById('t_spray').innerHTML    = (today.sprays||0) + '<span class="unit"> 件</span>';
    document.getElementById('t_pause').innerHTML    = (today.pauses||0) + '<span class="unit"> 回</span>';
    document.getElementById('t_uptime').textContent = s.uptime_str;
    const tEl = document.getElementById('t_temp');
    tEl.textContent = (s.cpu_temp == null ? 'N/A' : s.cpu_temp + '℃');
    tEl.className = 'val ' + tempClass(s.cpu_temp, s.temp_warn, s.temp_hot);

    // 棒グラフ（最大値でスケーリング。描画はブラウザ側なのでPiは無負荷）
    const days = s.days;
    const maxv = Math.max(1, ...days.map(d => (d.detections||0) + (d.sprays||0)));
    document.getElementById('bars').innerHTML = days.map(d => {
      const det = d.detections||0, spr = d.sprays||0, tot = det+spr;
      const h = Math.round((tot / maxv) * 110);
      const dh = tot ? Math.round(h * det / tot) : 0;
      const sh = h - dh;
      const md = d.date.slice(5);
      return '<div class="bar-col"><div class="bar-num">'+(tot||'')+'</div>'
        + '<div class="bar-stack" style="height:'+h+'px">'
        + '<div class="bar-detect" style="height:'+dh+'px"></div>'
        + '<div class="bar-spray" style="height:'+sh+'px"></div>'
        + '</div><div class="bar-date">'+md+'</div></div>';
    }).join('');

    // 明細テーブル（新しい日付を上に）
    document.getElementById('tbody').innerHTML = days.slice().reverse().map(d =>
      '<tr><td>'+d.date+'</td><td>'+d.armed_str+'</td><td>'+d.detections+'</td><td>'+d.sprays
      +'</td><td>'+d.pauses+'</td><td>'+d.resumes+'</td><td>'+d.offs+'</td><td>'+d.startups+'</td></tr>'
    ).join('');
  } catch (e) {
    document.getElementById('t_armed').textContent = '接続不可';
  }
}
refresh();
setInterval(refresh, 10000);
</script>
</body>
</html>
"""


@app.route("/dashboard")
def dashboard():
    return render_template_string(DASHBOARD)


# ============================================================
#  エントリーポイント
# ============================================================
def main():
    print("=" * 50)
    print("  🐱 猫撃退システム 起動")
    print(f"  Web操作: http://<このPiのIP>:{PORT}/ にアクセス")
    print("=" * 50)

    # 終了時に必ずポンプ・ブザーを止めるための保険（異常終了・systemd停止など）
    def _cleanup_gpio():
        try:
            relay_off()
            buzzer.off()
        except Exception:
            pass

    atexit.register(_cleanup_gpio)

    def _on_sigterm(signum, frame):
        # systemctl stop 等で送られるSIGTERMを拾い、確実に停止してから終了
        state.log("SIGTERM 受信：安全に停止します")
        shutdown_event.set()
        stats.flush()
        _cleanup_gpio()
        os._exit(0)

    signal.signal(signal.SIGTERM, _on_sigterm)

    # 新しい起動イベントを記録する前に、前回までのログを復元
    restore_from_log()

    stats.incr("startups")
    log_event("startup")

    # 監視スレッド・安全ウォッチドッグを開始
    threading.Thread(target=monitoring_loop, daemon=True).start()
    threading.Thread(target=watchdog_loop, daemon=True).start()
    camera.start()  # カメラは任意。無くても監視・散水はそのまま動く

    try:
        # reloader/デバッガはスレッド二重起動やGPIO競合の原因になるため無効化
        app.run(host=HOST, port=PORT, debug=False, use_reloader=False, threaded=True)
    except KeyboardInterrupt:
        state.log("Ctrl+C：終了します")
    finally:
        shutdown_event.set()
        buzzer.off()
        relay_off()
        time.sleep(0.2)


if __name__ == "__main__":
    main()
