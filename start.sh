#!/usr/bin/env bash
# 猫撃退システムを起動する（systemd の cat_deterrent サービス）
#   使い方: bash start.sh
set -u

SERVICE=cat_deterrent
PORT=5000

if ! systemctl cat "$SERVICE" >/dev/null 2>&1; then
    echo "❌ $SERVICE サービスが登録されていません。先に README の「自動起動（systemd）」を実施してください。"
    exit 1
fi

if systemctl is-active --quiet "$SERVICE"; then
    echo "ℹ️  既に起動しています。コードを反映したい場合は bash restart.sh を使ってください。"
else
    echo "▶️  起動します..."
    sudo systemctl start "$SERVICE"
    sleep 3  # 起動直後に落ちていないかを見るため少し待つ
fi

if systemctl is-active --quiet "$SERVICE"; then
    echo "✅ 稼働中です"
    echo "   操作画面      : http://$(hostname -I | awk '{print $1}'):$PORT/"
    echo "   ダッシュボード: http://$(hostname -I | awk '{print $1}'):$PORT/dashboard"
    echo "   ログを流し見  : journalctl -u $SERVICE -f"
else
    echo "❌ 起動に失敗しました。直近のログ:"
    sudo journalctl -u "$SERVICE" -n 30 --no-pager
    exit 1
fi
