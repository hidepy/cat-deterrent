#!/usr/bin/env bash
# 猫撃退システムを再起動する（コードを更新したら実行。停止中なら普通に起動する）
#   使い方: bash restart.sh
set -u

SERVICE=cat_deterrent
PORT=5000

if ! systemctl cat "$SERVICE" >/dev/null 2>&1; then
    echo "❌ $SERVICE サービスが登録されていません。先に README の「自動起動（systemd）」を実施してください。"
    exit 1
fi

# 構文エラーのまま再起動して止まりっぱなしになるのを防ぐ（その場合は今のプロセスを残す）
SCRIPT="$(cd "$(dirname "$0")" && pwd)/cat_deterrent.py"
if [ -f "$SCRIPT" ] && ! python3 -m py_compile "$SCRIPT"; then
    echo "❌ cat_deterrent.py に構文エラーがあるため、再起動を中止しました（稼働中のものはそのまま）"
    exit 1
fi

echo "🔄 再起動します..."
sudo systemctl restart "$SERVICE"
sleep 3  # 起動直後に落ちていないかを見るため少し待つ

if systemctl is-active --quiet "$SERVICE"; then
    echo "✅ 稼働中です"
    echo "   操作画面      : http://$(hostname -I | awk '{print $1}'):$PORT/"
    echo "   ダッシュボード: http://$(hostname -I | awk '{print $1}'):$PORT/dashboard"
    echo "   ログを流し見  : journalctl -u $SERVICE -f"
else
    echo "❌ 再起動に失敗しました。直近のログ:"
    sudo journalctl -u "$SERVICE" -n 30 --no-pager
    exit 1
fi
