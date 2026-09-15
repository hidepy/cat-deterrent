# 🐱 cat-deterrent

Raspberry Pi を使った**猫撃退システム**。PIRセンサーで動体を検知すると、警告ビープを鳴らしてからダイヤフラムポンプで散水し、猫を庭から遠ざけます。Flask 製の Web UI とダッシュボードで、遠隔操作と稼働状況の確認ができます。

> A Raspberry Pi–based cat deterrent: a PIR motion sensor triggers a warning beep and a water spray, controlled and monitored through a Flask web UI & dashboard.

---

## ✨ 主な機能

- **自動散水**：動体検知 → ビープ(1秒) → 1秒待機 → 散水 → クールタイム(5秒)
- **Web UI**（スマホ対応）：`ON` / `一時OFF`（5分停止して自動復帰）/ `OFF`（アプリ終了）
- **テストポンプ駆動**ボタン：呼び水・動作確認用にポンプだけを数秒回す（ビープ・検知とは独立）
- **ダッシュボード**：日別の稼働時間・検知/散水件数・一時OFF回数・CPU温度・棒グラフ
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

### 配線（BCM番号）

| デバイス | 信号ピン | Pi のピン |
|---|---|---|
| PIRセンサー OUT | GPIO 18 | 12番ピン |
| リレー IN | GPIO 17 | 11番ピン |
| ブザー I/O | GPIO 27 | 13番ピン |

- ブザーは Pi の GPIO に直結可能（VCC=5V、GND=GND、I/O=GPIO27）。ローレベルトリガのため、コード内では `active_high=False` で扱っています。
- リレーは標準の `active_high=True` を想定。ローアクティブ基板の場合は `RELAY_ACTIVE_HIGH = False` に変更してください。

---

## 📦 セットアップ

Raspberry Pi OS（Bookworm 以降）では、`pip` の直接インストールが制限されています（PEP 668）。**`apt` での導入を推奨**します。

```bash
sudo apt update
sudo apt install -y python3-flask python3-gpiozero python3-lgpio
```

> `gpiozero` と `lgpio` は Raspberry Pi OS に最初から入っていることが多く、実質 Flask を足すだけで済む場合があります。

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

---

## 🖥️ 画面 / API

| パス | 内容 |
|---|---|
| `/` | 操作画面（ON / 一時OFF / OFF / テストポンプ、直近ログ、本日の件数） |
| `/dashboard` | ダッシュボード（日別集計・CPU温度・グラフ・明細表） |
| `POST /api/on` `POST /api/pause` `POST /api/off` | 状態制御 |
| `POST /api/test_pump` | テストポンプ駆動 |
| `GET /api/status` | 現在状態・直近ログ・本日の件数（JSON） |
| `GET /api/stats` | 日別集計・CPU温度・連続稼働時間（JSON） |

---

## ⚙️ 主な設定値（`cat_deterrent.py` 冒頭）

| 定数 | 意味 | デフォルト |
|---|---|---|
| `BEEP_DURATION` | ビープの長さ | 1.0 秒 |
| `WAIT_AFTER_BEEP` | ビープ後、散水までの待機 | 1.0 秒 |
| `SPRAY_DURATION` | 散水の長さ | 4.0 秒 |
| `COOLDOWN` | 散水後のクールタイム | 5.0 秒 |
| `PAUSE_DURATION` | 「一時OFF」の停止時間 | 300 秒（5分） |
| `TEST_PUMP_DURATION` | テストポンプの駆動時間 | 5.0 秒 |
| `STATS_FLUSH_INTERVAL` | 集計のディスク書き込み間隔 | 60 秒 |

---

## 📁 生成されるデータファイル

| ファイル | 内容 |
|---|---|
| `events.jsonl` | 生イベントログ（1行1JSON） |
| `stats.json` | 日別集計 |

> これらは Pi 上の実行時に自動生成される**永続データ**です。`.gitignore` で除外済み。フォルダを丸ごと同期する際は**上書き・削除しないよう**ご注意ください（`git pull` は安全、`git clean -fdx` は禁物）。

---

## 🔒 セキュリティに関する注意

- Web UI に**認証はありません**。同じLAN内の端末なら誰でも操作できます。
- **インターネットへ晒さないでください**（ルーターのポート開放・DMZ は非推奨）。外部からアクセスしたい場合は Tailscale などの VPN 経由を推奨します。

---

## 📝 ライセンス

[MIT License](LICENSE)
