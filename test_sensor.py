import time
from datetime import datetime
from gpiozero import MotionSensor, OutputDevice

# --- ピンアサイン ---
PIR_PIN = 18    # 人感センサー OUT (GPIO 18 / 12番ピン)
RELAY_PIN = 17  # リレーモジュール IN (GPIO 17 / 11番ピン)

# デバイスの初期化
# ※リレーは通常ローアクティブ（信号LOWでON）の基板が多いですが、
# まずは標準設定(active_high=True)で動作確認します。
pir = MotionSensor(PIR_PIN)
relay = OutputDevice(RELAY_PIN, active_high=True, initial_value=False)

print("=" * 45)
print("  赤外線センサー 動作確認テスト")
print("  センサーの前で手を振ってみてください")
print("  終了するには [Ctrl + C] を押してください")
print("=" * 45)

# センサーが安定するまで少し待機
print("センサー初期化中（約3秒）...")
time.sleep(3)
print("監視スタンバイ完了！\n")

try:
    while True:
        # 動体を感知するまでここで待機
        pir.wait_for_motion()
        now_str = datetime.now().strftime('%H:%M:%S')
        print(f"[{now_str}] ★ 動体を検知しました！ (リレーON)")
        
        # リレーを3秒だけ動かしてみる（カチッと音がするか確認）
        relay.on()
        time.sleep(3.0)
        relay.off()
        print(f"[{now_str}] ── リレーOFF")

        # センサーがOFFに戻るまで待機
        pir.wait_for_no_motion()
        print(f"[{datetime.now().strftime('%H:%M:%S')}] センサー反応終了（待機状態に戻ります）\n")

except KeyboardInterrupt:
    print("\nテストを終了しました。")
finally:
    relay.off()
