# 🐱 cat-deterrent

Raspberry Pi を使った**猫撃退システム**。PIRセンサーで動体を検知すると、警告ビープを鳴らしてからダイヤフラムポンプで散水し、猫を庭から遠ざけます。Flask 製の Web UI とダッシュボードで、遠隔操作と稼働状況の確認ができます。

> A Raspberry Pi–based cat deterrent: a PIR motion sensor triggers a warning beep and a water spray, controlled and monitored through a Flask web UI & dashboard.

---

## ✨ 主な機能

- **自動散水**：動体検知 → 📷撮影 → 0.5秒の溜め → ビープ(1秒) → 1秒待機 → 散水（📷撮影） → クールタイム(5秒)
- **昼間の誤検知フィルタ**：昼（カメラ画像が明るい時）は PIR 反応後に 0.3 秒間隔で2コマ撮り、画面に動きが無ければ日光などによる誤検知とみなして散水しない。夜・カメラ不調時は PIR のみで判定。**お試しモード**（判定を記録するだけで散水は止めない）あり
- **判定レビュー画面**（`/review`）：昼の判定ごとに ①検知 ②0.3秒後 ③散水 の写真を並べ、「🐱猫いた／🚫いなかった」を付けると見逃し率・誤検知カット率を集計。しきい値を変えた場合の結果もその場で試算できる
- **散水しすぎ防止**：10分に5回以上散水が続いたら、次の検知までの間隔を 5→10分 に延長（最大10分＝1時間に約6回まで。ポンプ負荷・水切れ対策）。前回の散水から30分あけば通常に戻る。操作画面の ON で即解除
- **撮影**（USBカメラ・任意）：検知の瞬間と散水の瞬間の2枚をJPEG圧縮して `photos/` に保存。カメラが無くても散水は通常どおり動作
- **Web UI**（スマホ対応）：`ON` / `一時OFF`（5分停止して自動復帰）/ `OFF`（アプリ終了）
- **テストポンプ駆動**ボタン：呼び水・動作確認用にポンプだけを数秒回す（ビープ・検知とは独立）
- **ダッシュボード**：直近3回分の撮影（検知・散水の2枚組）、日別の稼働時間・検知/散水件数・一時OFF回数・CPU温度・棒グラフ
- **ロギング**：全イベントを `events.jsonl` に、日別集計を `stats.json` に永続化。再起動しても UI に復元
- **Piに優しい設計**：集計はイベント発生時に加算、グラフ描画はブラウザ側、SD保護のため書き込みはまとめ書き

---

## 🔌 ハードウェア構成

| 部品 | 役割 |
|---|---|
| Raspberry Pi 4 | コントローラ |
| PIRセンサー | 動体検知 |
| リレーモジュール | ダイヤフラムポンプの電源ON/OFF |
| ダイヤフラムポンプ + ホース | 散水 |
| アクティブブザー（3.3〜5V / ローレベルトリガ） | 散水直前の警告音 |
| USBカメラ（UVC対応・任意） | 検知時・散水時の撮影 |

### 配線（BCM番号）

| デバイス | 信号ピン | Pi のピン |
|---|---|---|
| PIRセンサー OUT | GPIO 18 | 12番ピン |
| リレー IN | GPIO 17 | 11番ピン |
| ブザー VCC | 3.3V | 1 or 17番ピン |
| ブザー GND | GND | — |
| ブザー I/O | GPIO 27 | 13番ピン |

- ブザーはローレベルトリガ（信号LOWで鳴る）。コードでは `active_high=False`（静音＝HIGH）で扱う。
- ⚠️ **ブザーのVCCは必ず 3.3V に**（5Vではない）。5Vで駆動すると、GPIOの「HIGH」は 3.3V しかないためブザーを OFF にしきれず**鳴りっぱなし**になる（電圧ミスマッチ）。3.3V駆動なら「HIGH＝完全OFF」となり正常動作する。
- **起動直後（アプリ未起動）の誤鳴き対策**：`/boot/firmware/config.txt` に **`gpio=27=op,dh`** を追記し、電源投入時からピンを HIGH（＝静音）に固定する。
- リレーは標準の `active_high=True` を想定。ローアクティブ基板の場合は `RELAY_ACTIVE_HIGH = False` に変更してください。

---

## 📦 セットアップ

Raspberry Pi OS（Bookworm 以降）では、`pip` の直接インストールが制限されています（PEP 668）。**`apt` での導入を推奨**します。

```bash
sudo apt update
sudo apt install -y python3-flask python3-gpiozero python3-lgpio python3-opencv
```

> `gpiozero` と `lgpio` は Raspberry Pi OS に最初から入っていることが多く、実質 Flask を足すだけで済む場合があります。
> `python3-opencv` はカメラ撮影用です。未導入でも撮影がスキップされるだけで、他の機能は動作します。
> USBカメラが認識されているかは `ls /dev/video*` で確認できます（通常は `/dev/video0`）。

---

## ▶️ 実行

```bash
cd ~/cat_system
python3 cat_deterrent.py
```

起動すると監視ループと Web サーバーが同時に立ち上がります。同じLAN内の端末から下記へアクセスしてください（IPは `hostname -I` で確認）。

```
http://<PiのIP>:5000/
```

- ポートは `cat_deterrent.py` 冒頭の `PORT`（デフォルト `5000`）で変更できます。
- GPIO で権限エラーが出る場合は `sudo python3 cat_deterrent.py` を試してください。

### 自動起動（任意 / systemd）

同梱の `cat_deterrent.service` を使うと、電源投入時に自動起動できます。
**`WorkingDirectory` / `ExecStart` のパスと `User` は、ご自身の環境に合わせて編集してください。**

```bash
sudo cp cat_deterrent.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now cat_deterrent
```

`Restart=on-failure` のため、Web の `OFF`（正常終了）ではちゃんと止まり、異常クラッシュ時だけ自動復帰します。

サービス登録後は、同梱のスクリプトで起動・再起動できます（稼働確認とURL表示まで行います）。

```bash
bash start.sh     # 起動（既に稼働中なら何もしない）
bash restart.sh   # 再起動（コード更新の反映用。構文エラーがあれば中止して現行を残す）
sudo systemctl stop cat_deterrent   # 停止
```

---

## 🖥️ 画面 / API

| パス | 内容 |
|---|---|
| `/` | 操作画面（ON / 一時OFF / OFF / テストポンプ、直近ログ、本日の件数） |
| `/dashboard` | ダッシュボード（直近の撮影・日別集計・CPU温度・グラフ・明細表） |
| `/review` | 判定レビュー（昼の動き判定の写真確認・正解ラベル付け・見逃し率の集計） |
| `GET /api/review?date=YYYY-MM-DD` | その日の昼の判定一覧（JSON） |
| `GET /api/review/labeled` | 全期間のラベル済み判定（JSON） |
| `POST /api/check_mode` | 昼間のカメラ判定モードの切り替え `{"mode": "trial" \| "enforce" \| "off"}` |
| `POST /api/review/label` | 正解ラベルの保存 `{"id": "YYYY-MM-DD/HHMMSS", "label": "cat" \| "none" \| "unsure" \| null}` |
| `GET /api/photos?limit=3` | 直近の撮影（検知＋散水の組、新しい順・最大20）（JSON） |
| `GET /photos/<日付>/<ファイル名>` | 撮影画像 |
| `POST /api/on` `POST /api/pause` `POST /api/off` | 状態制御 |
| `POST /api/test_pump` | テストポンプ駆動 |
| `GET /api/status` | 現在状態・直近ログ・本日の件数（JSON） |
| `GET /api/stats` | 日別集計・CPU温度・連続稼働時間（JSON） |

---

## ⚙️ 主な設定値（`cat_deterrent.py` 冒頭）

| 定数 | 意味 | デフォルト |
|---|---|---|
| `PRE_BEEP_DELAY` | 検知（1枚目撮影）からビープまでの溜め | 0.5 秒 |
| `BEEP_DURATION` | ビープの長さ | 1.0 秒 |
| `WAIT_AFTER_BEEP` | ビープ後、散水までの待機 | 1.0 秒 |
| `SPRAY_DURATION` | 散水の長さ | 4.0 秒 |
| `COOLDOWN` | 散水後のクールタイム | 5.0 秒 |
| `PAUSE_DURATION` | 「一時OFF」の停止時間 | 300 秒（5分） |
| `TEST_PUMP_DURATION` | テストポンプの駆動時間 | 5.0 秒 |
| `STATS_FLUSH_INTERVAL` | 集計のディスク書き込み間隔 | 60 秒 |
| `CAMERA_ENABLED` | カメラ撮影のON/OFF | `True` |
| `CAMERA_WIDTH` / `CAMERA_HEIGHT` | 撮影解像度（大きい画像は幅に合わせて縮小）。16:9センサーで4:3を指定すると左右が切れて画角が狭くなるため16:9 | 640 × 360 |
| `CAMERA_ROTATE` | 保存時の回転（0 / 90 / 180 / 270・時計回り）。カメラを逆さに付けた場合は 180 | 180 |
| `PHOTO_JPEG_QUALITY` | JPEG画質（下げるほど小さい） | 70 |
| `SPRAY_SHOT_DELAY` | ポンプONから2枚目を撮るまでの遅れ | 0.5 秒 |
| `PHOTO_RETENTION_DAYS` | 画像の保存日数（古い日付フォルダは自動削除） | 30 日 |
| `MOTION_CHECK_MODE` | 昼間の誤検知フィルタの初期値。`"trial"`＝判定を記録するだけ（散水は止めない）／`"enforce"`＝動きなしなら見送り／`"off"`。**ダッシュボードで切り替え可能**（画面で選ぶと `runtime_settings.json` の値が優先） | `"trial"` |
| `MOTION_CHECK_INTERVAL` | 動き判定の2コマの撮影間隔 | 0.3 秒 |
| `MOTION_PIXEL_THRESHOLD` | 1ピクセルを「変化あり」とみなす明るさ差（0-255） | 25 |
| `MOTION_MIN_RATIO` | 「動きあり」とみなす変化ピクセルの割合 | 0.005（0.5%） |
| `REJECT_COOLDOWN` | 本番モードで見送った後、再判定までの待ち | 3.0 秒 |
| `SPRAY_RATE_WINDOW` / `SPRAY_RATE_MAX` | この期間にこの回数以上散水したら間隔を延長 | 600 秒 / 5 回 |
| `THROTTLE_BASE` / `THROTTLE_MAX` | 延長する間隔（段階ごとに2倍）／その上限 | 300 秒 / 600 秒 |
| `THROTTLE_CALM` | 前回の散水からこの時間あいたら（PIRが落ち着いた）通常に戻す | 1800 秒 |
| `DAY_BRIGHTNESS` / `NIGHT_BRIGHTNESS` | 平均輝度がこれ以上で昼／以下で夜（間は直前の判定を維持） | 60 / 40 |
| `BRIGHTNESS_INTERVAL` | 明るさを測る間隔 | 30 秒 |

### 誤検知フィルタの試し方・しきい値調整
1. `MOTION_CHECK_MODE = "trial"`（お試し）で数日動かす。昼の検知ごとに判定と写真が記録されるが、散水は今までどおり行う
2. `/review` で写真を見ながら「🐱 猫いた／🚫 いなかった」を付ける
3. 集計表の **⚠️見逃し**（猫がいたのに「動きなし」判定）が少なく、**誤検知を防止** が多ければ、ダッシュボードの「☀️ 昼間のカメラ判定」で **✅ 本番** に切り替え。スライダーで「しきい値をいくつにすればよいか」も試算できる
- ダッシュボードの「判定モード」に現在の明るさが出ます。夕方・明け方の値を見て `DAY_BRIGHTNESS` / `NIGHT_BRIGHTNESS` を合わせてください。

---

## 📁 生成されるデータファイル

| ファイル | 内容 |
|---|---|
| `events.jsonl` | 生イベントログ（1行1JSON） |
| `stats.json` | 日別集計 |
| `photos/YYYY-MM-DD/HHMMSS_1_detect.jpg` | 検知の瞬間の画像 |
| `photos/YYYY-MM-DD/HHMMSS_1b_check.jpg` | 昼の動き判定用、検知の0.3秒後の画像 |
| `photos/YYYY-MM-DD/HHMMSS_2_spray.jpg` | 散水の瞬間の画像（`HHMMSS` は検知時刻なので同じ組になる） |
| `photos/YYYY-MM-DD/HHMMSS_0_reject_a.jpg` / `_b.jpg` | 本番モードで見送った時の2コマ（ダッシュボードには出ない） |
| `review_labels.json` | 判定レビュー画面で付けた正解ラベル |
| `runtime_settings.json` | ダッシュボードで切り替えた設定（昼間のカメラ判定モード） |

`events.jsonl` の `detect` / `spray` には、撮影できた場合 `"photo": "photos/..."` が付きます。

> これらは Pi 上の実行時に自動生成される**永続データ**です。`.gitignore` で除外済み。フォルダを丸ごと同期する際は**上書き・削除しないよう**ご注意ください（`git pull` は安全、`git clean -fdx` は禁物）。

---

## 🔒 セキュリティに関する注意

- Web UI に**認証はありません**。同じLAN内の端末なら誰でも操作できます。
- **インターネットへ晒さないでください**（ルーターのポート開放・DMZ は非推奨）。外部からアクセスしたい場合は Tailscale などの VPN 経由を推奨します。

---

## 📝 ライセンス

[MIT License](LICENSE)
